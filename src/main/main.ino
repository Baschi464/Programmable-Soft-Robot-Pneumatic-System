#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ==========================================================================
//   8-CHANNEL PNEUMATIC CONTROLLER - SMC (Sliding Mode Control)
//   Shared pumps, independent channels with per-channel SMC gains
// ==========================================================================

// --- CONFIGURATION ---
const float DEFAULT_TARGET = 0.0;

// --- SMC DEFAULT GAINS (Applied to all channels initially) ---
const float DEF_LAMBDA       = 2.5;    // Sliding surface gain ("Braking Strength" — increase if overshooting)
const float DEF_PHI           = 8.0;    // Boundary layer thickness (increase for smoother/slower approach)
const float DEF_K_GAIN        = 1000.0; // Max pulse duration in ms (base speed)
const float DEF_P_TOLERANCE   = 0.5;    // Deadband (kPa)
const float DEF_PASSIVE_THR   = 0.4;    // If |sat| < this, use passive vent instead of pump
const float DEF_VENT_BOOST    = 1.5;    // Multiplier for passive vent duration (slower than active)
const float DEF_ATMOSPHERIC_P = 0.0;    // Approx atmospheric pressure (kPa)

// --- TIMING & PULSE LIMITS ---
const int REST_TIME       = 200;  // Settling time after fine pulse (ms)
const int COARSE_REST     = 10;   // Minimal rest after coarse (saturated) pulse (ms)
const int MIN_PULSE       = 10;   // Shortest possible valve opening (ms)
const int MIN_PUMP_PULSE  = 30;   // Don't turn on pump for tiny pulses to save motor
const int MAX_PULSE       = 2000; // Safety cap (ms)

// --- SENSOR SETTINGS ---
const float ADS_BIT_VOLTAGE = 0.0001875;
const float V_OFFSET    = 0.5;
const float V_FULLSCALE = 4.5;
const float P_MIN_KPA   = -100.0;
const float P_MAX_KPA   = 100.0;
const float V_SPAN = V_FULLSCALE - V_OFFSET;
const float P_SPAN = P_MAX_KPA - P_MIN_KPA;
// --- INDIVIDUAL SENSOR OFFSETS (Calibration) ---
float SENSOR_OFFSETS[8] = { -0.91, -0.68, -0.65, 0.0, 0.0, 0.0, 0.0, 0.0 };

// --- PINS (USER TO CONFIGURE) ---
// Shared Pumps
const int PIN_PUMP_POS = 12;
const int PIN_PUMP_NEG = 13;

// Valve Arrays (Index 0 = Channel 1, etc.)
const int VALVES_POS[8] = { 2, 40, 40, 40, 40, 40, 40, 40 };
const int VALVES_NEG[8] = { 35, 40, 40, 40, 40, 40, 40, 40 };

// --- STATE ENUM ---
enum ControlState {
  STATE_IDLE,
  STATE_INFLATING,          // Positive valve open (pump requested if pulse > MIN_PUMP_PULSE)
  STATE_DEFLATING_ACTIVE,   // Negative valve open + vacuum pump
  STATE_DEFLATING_PASSIVE,  // Negative valve open only (physics vents)
  STATE_RESTING             // All valves closed, waiting for sensor to settle
};

// --- CONTROL MODE (for telemetry / logging) ---
enum ControlMode {
  MODE_IDLE,
  MODE_COARSE_INF,   // Saturated positive (|sat| >= 0.95)
  MODE_COARSE_DEF,   // Saturated negative
  MODE_FINE_INF,     // Pulsing positive
  MODE_FINE_DEF,     // Pulsing negative (active)
  MODE_PASSIVE_DEF   // Pulsing negative (passive vent)
};

// --- CHANNEL STRUCT ---
struct ChannelControl {
  // Pin Config
  int pinValvePos;
  int pinValveNeg;

  // Dynamic Variables
  float targetPressure;
  float currentPressure;
  ControlState currentState;
  ControlMode  currentMode;
  unsigned long stateStartTime;
  unsigned long currentPulseDuration;

  // Pump Request Flags (for the shared Pump Manager)
  bool reqPosPump;
  bool reqNegPump;

  // "Off" flag — when true the channel is ignored (no actuation)
  // Set when the GUI sends "off" instead of a numeric target.
  bool isOff;

  // --- SMC Internal State (per channel) ---
  float prev_error;
  unsigned long last_loop_time;
  float last_sat;           // Stored for rest-time decision

  // --- Per-Channel SMC Gains ---
  float lambda_smc;         // Sliding surface gain
  float phi;                // Boundary layer thickness
  float k_gain;             // Max pulse duration (ms)
  float p_tolerance;        // Deadband (kPa)
  float passive_threshold;  // |sat| below this → passive vent
  float vent_boost_gain;    // Multiplier for passive vent pulse
  float atmospheric_p;      // Reference for passive vent decision
};

// --- GLOBAL OBJECTS ---
ChannelControl channels[8];
Adafruit_ADS1115 ads1; // Address 0x48 (Channels 0-3)
Adafruit_ADS1115 ads2; // Address 0x49 (Channels 4-7)

// --- COMMUNICATION VARIABLES ---
const byte numChars = 128;
char receivedChars[numChars];
boolean newData = false;
unsigned long lastTelemetryTime = 0;
const long TELEMETRY_INTERVAL = 100; // 10 Hz updates to Python

// ==========================================================================
//   HELPER FUNCTIONS
// ==========================================================================

// --- Read Pressure (Selects correct ADS module) ---
float readPressure(int chIndex) {
  int16_t raw;

  if (chIndex < 4) {
    raw = ads1.readADC_SingleEnded(chIndex);
  } else {
    raw = ads2.readADC_SingleEnded(chIndex - 4);
  }

  float voltage = raw * ADS_BIT_VOLTAGE;
  if (voltage < 0.2) return -1000.0; // Wire break / disconnected
  if (voltage < V_OFFSET) voltage = V_OFFSET;

  float kpa = (P_MIN_KPA + ((voltage - V_OFFSET) / V_SPAN) * P_SPAN) - SENSOR_OFFSETS[chIndex];
  if (kpa < -90.0) return -1000.0;
  return kpa;
}

// --- Mode Name (for logging) ---
String getModeName(ControlMode m) {
  switch (m) {
    case MODE_IDLE:        return "IDLE";
    case MODE_COARSE_INF:  return "COARSE_+";
    case MODE_COARSE_DEF:  return "COARSE_-";
    case MODE_FINE_INF:    return "FINE_+";
    case MODE_FINE_DEF:    return "FINE_-";
    case MODE_PASSIVE_DEF: return "PASSIVE_V";
    default:               return "?";
  }
}

// --- Serial Parsing (Matches Python GUI Format) ---
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) ndx = numChars - 1;
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {
  char *strtokIndx;
  strtokIndx = strtok(receivedChars, ",");

  if (strtokIndx != NULL) {
    if (strcmp(strtokIndx, "off") == 0) {
      channels[0].isOff = true;
    } else {
      if (channels[0].isOff) {
        // Transitioning off → on: reset SMC state to avoid derivative spike
        channels[0].last_loop_time = millis();
        channels[0].prev_error = 0.0;
      }
      channels[0].isOff = false;
      channels[0].targetPressure = atof(strtokIndx);
    }
  }
  for (int i = 1; i < 8; i++) {
    strtokIndx = strtok(NULL, ",");
    if (strtokIndx != NULL) {
      if (strcmp(strtokIndx, "off") == 0) {
        channels[i].isOff = true;
      } else {
        if (channels[i].isOff) {
          // Transitioning off → on: reset SMC state to avoid derivative spike
          channels[i].last_loop_time = millis();
          channels[i].prev_error = 0.0;
        }
        channels[i].isOff = false;
        channels[i].targetPressure = atof(strtokIndx);
      }
    }
  }
}

// ==========================================================================
//   SETUP
// ==========================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  Wire.begin();
  Wire.setClock(100000); // 100 kHz I2C (safe against noise / bus freeze)

  // Pump Pins
  pinMode(PIN_PUMP_POS, OUTPUT); digitalWrite(PIN_PUMP_POS, LOW);
  pinMode(PIN_PUMP_NEG, OUTPUT); digitalWrite(PIN_PUMP_NEG, LOW);

  unsigned long now = millis();

  // Initialize Channels
  for (int i = 0; i < 8; i++) {
    channels[i].pinValvePos = VALVES_POS[i];
    channels[i].pinValveNeg = VALVES_NEG[i];
    pinMode(channels[i].pinValvePos, OUTPUT); digitalWrite(channels[i].pinValvePos, LOW);
    pinMode(channels[i].pinValveNeg, OUTPUT); digitalWrite(channels[i].pinValveNeg, LOW);

    channels[i].currentState       = STATE_IDLE;
    channels[i].currentMode        = MODE_IDLE;
    channels[i].targetPressure     = DEFAULT_TARGET;
    channels[i].currentPressure    = 0.0;
    channels[i].stateStartTime     = 0;
    channels[i].currentPulseDuration = 0;
    channels[i].reqPosPump         = false;
    channels[i].reqNegPump         = false;
    channels[i].isOff              = false;

    // SMC internal state
    channels[i].prev_error     = 0.0;
    channels[i].last_loop_time = now;
    channels[i].last_sat       = 0.0;

    // --- Assign Default SMC Gains (can be overridden per channel below) ---
    channels[i].lambda_smc        = DEF_LAMBDA;
    channels[i].phi               = DEF_PHI;
    channels[i].k_gain            = DEF_K_GAIN;
    channels[i].p_tolerance       = DEF_P_TOLERANCE;
    channels[i].passive_threshold = DEF_PASSIVE_THR;
    channels[i].vent_boost_gain   = DEF_VENT_BOOST;
    channels[i].atmospheric_p     = DEF_ATMOSPHERIC_P;
  }

  // ==========================================================================
  // --- EXAMPLE: CUSTOM SMC TUNING FOR A SPECIFIC CHANNEL ---
  //
  // channels[3].lambda_smc        = 3.0;   // Stronger braking (default 2.5)
  // channels[3].phi               = 10.0;  // Wider boundary → smoother (default 8.0)
  // channels[3].k_gain            = 800.0; // Lower max pulse (default 1000)
  // channels[3].p_tolerance       = 0.3;   // Tighter deadband (default 0.5)
  // channels[3].passive_threshold = 0.5;   // More passive venting (default 0.4)
  // channels[3].vent_boost_gain   = 2.0;   // Longer passive pulses (default 1.5)
  //
  // All other channels still use the defaults defined above.
  // ==========================================================================

  // Initialize Sensors
  if (!ads1.begin(0x48)) { Serial.println("ADS1 Failed"); while (1); }
  if (!ads2.begin(0x49)) { Serial.println("ADS2 Failed"); while (1); }

  ads1.setGain(GAIN_TWOTHIRDS);
  ads2.setGain(GAIN_TWOTHIRDS);
  ads1.setDataRate(RATE_ADS1115_860SPS);
  ads2.setDataRate(RATE_ADS1115_860SPS);

  Serial.println("<ARDUINO_READY_8CH>");
}

// ==========================================================================
//   MAIN LOOP
// ==========================================================================
void loop() {

  // 1. COMMUNICATION (Receive Targets from Python GUI)
  recvWithStartEndMarkers();
  if (newData == true) {
    parseData();
    newData = false;
  }

  // 2. CONTROL LOOP (Iterate all 8 channels)
  bool anyChannelNeedsPosPump = false;
  bool anyChannelNeedsNegPump = false;

  for (int i = 0; i < 8; i++) {
    ChannelControl &ch = channels[i];

    // A. Read Sensor
    ch.currentPressure = readPressure(i);

    // --- SAFETY: Sensor disconnect ---
    if (ch.currentPressure == -1000.0) {
      digitalWrite(ch.pinValvePos, LOW);
      digitalWrite(ch.pinValveNeg, LOW);
      ch.currentState = STATE_IDLE;
      ch.currentMode  = MODE_IDLE;
      continue;
    }

    // --- OFF CHANNEL: no keypoints in current action → skip control entirely ---
    // Valves stay closed, no pump request, but sensor is still read for telemetry.
    if (ch.isOff) {
      digitalWrite(ch.pinValvePos, LOW);
      digitalWrite(ch.pinValveNeg, LOW);
      ch.currentState = STATE_IDLE;
      ch.currentMode  = MODE_IDLE;
      ch.prev_error   = 0.0;
      continue;
    }

    // Reset pump flags for this cycle
    ch.reqPosPump = false;
    ch.reqNegPump = false;

    // B. State Machine
    switch (ch.currentState) {

      // ================================================================
      //  IDLE — Compute SMC law, decide next action
      // ================================================================
      case STATE_IDLE: {
        // --- Time delta ---
        unsigned long current_time = millis();
        float dt = (current_time - ch.last_loop_time) / 1000.0; // seconds
        if (dt <= 0.001) dt = 0.001; // protect against zero-division

        float error = ch.targetPressure - ch.currentPressure;

        // --- Deadband ---
        if (abs(error) <= ch.p_tolerance) {
          ch.currentMode  = MODE_IDLE;
          ch.prev_error   = error;
          ch.last_loop_time = current_time;
          break; // nothing to do
        }

        // --- Error derivative ---
        float error_dot = (error - ch.prev_error) / dt;

        // --- Sliding surface ---
        float s = (ch.lambda_smc * error) + error_dot;

        // --- Saturation function (clamp to [-1, +1]) ---
        float sat = s / ch.phi;
        if (sat >  1.0) sat =  1.0;
        if (sat < -1.0) sat = -1.0;

        // --- Pulse width ---
        unsigned long pulse_width = (unsigned long)(fabs(sat) * ch.k_gain);
        if (pulse_width > (unsigned long)MAX_PULSE) pulse_width = MAX_PULSE;
        ch.last_sat = sat;

        // ============================================================
        //  INFLATION  (sat > 0)
        // ============================================================
        if (sat > 0) {
          if (pulse_width > (unsigned long)MIN_PULSE) {
            ch.currentMode = (fabs(sat) >= 0.95) ? MODE_COARSE_INF : MODE_FINE_INF;

            digitalWrite(ch.pinValveNeg, LOW);
            digitalWrite(ch.pinValvePos, HIGH);
            if (pulse_width > (unsigned long)MIN_PUMP_PULSE) ch.reqPosPump = true;

            ch.currentPulseDuration = pulse_width;
            ch.currentState   = STATE_INFLATING;
            ch.stateStartTime = millis();
          }
        }
        // ============================================================
        //  DEFLATION  (sat < 0)
        // ============================================================
        else {
          if (pulse_width > (unsigned long)MIN_PULSE) {
            // Decide passive vs active venting
            bool usePassive = (fabs(sat) < ch.passive_threshold) &&
                              (ch.currentPressure > ch.atmospheric_p + 2.0);

            if (usePassive) {
              // --- Passive Vent (valve only, no pump) ---
              ch.currentMode = MODE_PASSIVE_DEF;
              digitalWrite(ch.pinValvePos, LOW);
              digitalWrite(ch.pinValveNeg, HIGH);

              ch.currentPulseDuration = (unsigned long)(pulse_width * ch.vent_boost_gain);
              if (ch.currentPulseDuration > (unsigned long)MAX_PULSE) ch.currentPulseDuration = MAX_PULSE;
              ch.currentState   = STATE_DEFLATING_PASSIVE;
              ch.stateStartTime = millis();
            } else {
              // --- Active Vacuum (valve + pump) ---
              ch.currentMode = (fabs(sat) >= 0.95) ? MODE_COARSE_DEF : MODE_FINE_DEF;
              digitalWrite(ch.pinValvePos, LOW);
              digitalWrite(ch.pinValveNeg, HIGH);
              if (pulse_width > (unsigned long)MIN_PUMP_PULSE) ch.reqNegPump = true;

              ch.currentPulseDuration = pulse_width;
              ch.currentState   = STATE_DEFLATING_ACTIVE;
              ch.stateStartTime = millis();
            }
          }
        }

        // Update SMC state for next iteration
        ch.prev_error     = error;
        ch.last_loop_time = current_time;
        break;
      }

      // ================================================================
      //  INFLATING — Positive valve open, pump on if needed
      // ================================================================
      case STATE_INFLATING:
        if (ch.currentPulseDuration > (unsigned long)MIN_PUMP_PULSE) ch.reqPosPump = true;

        if (millis() - ch.stateStartTime >= ch.currentPulseDuration) {
          digitalWrite(ch.pinValvePos, LOW);
          ch.currentState   = STATE_RESTING;
          ch.stateStartTime = millis();
        }
        break;

      // ================================================================
      //  DEFLATING (Active) — Negative valve + vacuum pump
      // ================================================================
      case STATE_DEFLATING_ACTIVE:
        if (ch.currentPulseDuration > (unsigned long)MIN_PUMP_PULSE) ch.reqNegPump = true;

        if (millis() - ch.stateStartTime >= ch.currentPulseDuration) {
          digitalWrite(ch.pinValveNeg, LOW);
          ch.currentState   = STATE_RESTING;
          ch.stateStartTime = millis();
        }
        break;

      // ================================================================
      //  DEFLATING (Passive) — Negative valve only, no pump
      // ================================================================
      case STATE_DEFLATING_PASSIVE:
        // No pump request — physics does the work
        if (millis() - ch.stateStartTime >= ch.currentPulseDuration) {
          digitalWrite(ch.pinValveNeg, LOW);
          ch.currentState   = STATE_RESTING;
          ch.stateStartTime = millis();
        }
        break;

      // ================================================================
      //  RESTING — All valves closed, wait for pressure to settle
      // ================================================================
      case STATE_RESTING: {
        digitalWrite(ch.pinValvePos, LOW);
        digitalWrite(ch.pinValveNeg, LOW);

        // Shorter rest when saturated (coarse), longer when fine-tuning
        unsigned long restDuration = (fabs(ch.last_sat) >= 0.95)
                                       ? (unsigned long)COARSE_REST
                                       : (unsigned long)REST_TIME;

        if (millis() - ch.stateStartTime >= restDuration) {
          ch.currentState = STATE_IDLE;
        }
        break;
      }
    } // end switch

    // Accumulate pump requests across channels
    if (ch.reqPosPump) anyChannelNeedsPosPump = true;
    if (ch.reqNegPump) anyChannelNeedsNegPump = true;
  } // end channel loop

  // 3. PUMP MANAGER (shared pumps — turn on if ANY channel requests)
  digitalWrite(PIN_PUMP_POS, anyChannelNeedsPosPump ? HIGH : LOW);
  digitalWrite(PIN_PUMP_NEG, anyChannelNeedsNegPump ? HIGH : LOW);

  // 4. TELEMETRY (Send to Python GUI)
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = millis();

    // Format: ACT:v1,v2,v3,v4,v5,v6,v7,v8
    Serial.print("ACT:");
    for (int i = 0; i < 8; i++) {
      if (channels[i].currentPressure == -1000.0) {
        Serial.print("ERR");
      } else {
        Serial.print(channels[i].currentPressure, 1);
      }
      if (i < 7) Serial.print(",");
    }
    Serial.println();
  }
}
