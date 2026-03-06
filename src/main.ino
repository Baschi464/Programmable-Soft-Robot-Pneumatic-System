#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

// ==========================================================================
//   8-CHANNEL PNEUMATIC CONTROLLER - SMC (Sliding Mode Control)
//   Shared pumps, independent channels with per-channel SMC gains
// ==========================================================================

// --- CONFIGURATION ---
const float DEFAULT_TARGET = 0.0;

// --- SMC DEFAULT GAINS (Applied to all channels initially) ---
const float DEF_LAMBDA        = 18.0;   // Sliding surface gain ("Braking Strength")
const float DEF_PHI           = 40.0;   // Boundary layer thickness
const float DEF_K_GAIN        = 100.0;   // Base Speed multiplier
const float DEF_KD_GAIN       = 0.0;    // Derivative multiplier
const float DEF_P_TOLERANCE   = 1.0;    // Deadband (kPa)
const float DEF_PASSIVE_THR   = 0.8;    // If |sat| < this, use passive vent instead of pump
const float DEF_VENT_GAIN     = 2.0;   // Multiplier for passive vent duration
const float DEF_ATMOSPHERIC_P = 0.5;    // Approx atmospheric pressure (kPa)

// --- TIMING & PULSE LIMITS ---
const int REST_TIME       = 200;  // Settle time for fine tuning (ms)
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
float SENSOR_OFFSETS[8] = { -0.73, -0.69, -0.72, -0.67, -0.64, -0.85, -0.58, -0.53 };

// --- PINS (USER TO CONFIGURE) ---
// Shared Pumps
const int PIN_PUMP_POS = 13;
const int PIN_PUMP_NEG = 12;

// Valve Arrays (Index 0 = Channel 1, etc.)
const int VALVES_POS[8] = { 42, 44, 46, 48, 50, 52, 51, 53 };
const int VALVES_NEG[8] = { 36, 34, 32, 30, 28, 26, 24, 22 };

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

  // --- Median Filter State (per channel) ---
  float p_history[3];
  bool history_initialized;

  // --- Per-Channel SMC Gains ---
  float lambda_smc;         // Sliding surface gain
  float phi;                // Boundary layer thickness
  float k_gain;             // Base Speed multiplier
  float kd_gain;            // Derivative multiplier
  float p_tolerance;        // Deadband (kPa)
  float passive_threshold;  // |sat| below this → passive vent
  float vent_gain;          // Multiplier for passive vent pulse
  float atmospheric_p;      // Reference for passive vent decision

  // --- Debug snapshot (recorded during IDLE decision, read at telemetry time) ---
  float dbg_error;
  float dbg_sat;
  unsigned long dbg_pulse;
  // currentMode already captures the action taken
};

// --- GLOBAL OBJECTS ---
ChannelControl channels[8];
Adafruit_ADS1115 ads1; // Address 0x48 (Channels 0-3)
Adafruit_ADS1115 ads2; // Address 0x49 (Channels 4-7)

// --- DEBUG FLAG  ---
const bool DEBUG_TELEMETRY = false;

// --- COMMUNICATION VARIABLES ---
const byte numChars = 128;
char receivedChars[numChars];
boolean newData = false;
unsigned long lastTelemetryTime = 0;
const long TELEMETRY_INTERVAL = 100; // 10 Hz updates to Python
unsigned long loopStartTime = 0;     // For measuring loop duration

// ==========================================================================
//   HELPER FUNCTIONS
// ==========================================================================

// --- Read Pressure (Selects correct ADS module & input) ---
//   Ch0→ads2[3], Ch1→ads2[2], Ch2→ads2[1], Ch3→ads2[0]
//   Ch4→ads1[0], Ch5→ads1[1], Ch6→ads1[2], Ch7→ads1[3]
float readPressure(int chIndex) {
  int16_t raw = 0;
  bool validChannel = true;

  switch (chIndex) {
    case 0: raw = ads2.readADC_SingleEnded(3); break;
    case 1: raw = ads2.readADC_SingleEnded(2); break;
    case 2: raw = ads2.readADC_SingleEnded(1); break;
    case 3: raw = ads2.readADC_SingleEnded(0); break;
    case 4: raw = ads1.readADC_SingleEnded(0); break;
    case 5: raw = ads1.readADC_SingleEnded(1); break;
    case 6: raw = ads1.readADC_SingleEnded(2); break;
    case 7: raw = ads1.readADC_SingleEnded(3); break;
    default: validChannel = false; break;
  }
  if (!validChannel) return -1000.0;

  float voltage = raw * ADS_BIT_VOLTAGE;
  if (voltage < 0.2) return -1000.0; // Wire break / disconnected
  if (voltage < V_OFFSET) voltage = V_OFFSET;

  float kpa = (P_MIN_KPA + ((voltage - V_OFFSET) / V_SPAN) * P_SPAN) - SENSOR_OFFSETS[chIndex];
  if (kpa < -90.0) return -1000.0;
  return kpa;
}

// --- Median-Filtered Pressure (per channel) ---
float getCleanPressure(int chIndex) {
  ChannelControl &ch = channels[chIndex];
  float raw_reading = readPressure(chIndex);
  if (raw_reading == -1000.0) return raw_reading;

  if (!ch.history_initialized) {
    ch.p_history[0] = ch.p_history[1] = ch.p_history[2] = raw_reading;
    ch.history_initialized = true;
  } else {
    ch.p_history[2] = ch.p_history[1];
    ch.p_history[1] = ch.p_history[0];
    ch.p_history[0] = raw_reading;
  }

  float median_val;
  if ((ch.p_history[0] <= ch.p_history[1] && ch.p_history[1] <= ch.p_history[2]) || 
      (ch.p_history[2] <= ch.p_history[1] && ch.p_history[1] <= ch.p_history[0])) {
    median_val = ch.p_history[1];
  } 
  else if ((ch.p_history[1] <= ch.p_history[0] && ch.p_history[0] <= ch.p_history[2]) || 
           (ch.p_history[2] <= ch.p_history[0] && ch.p_history[0] <= ch.p_history[1])) {
    median_val = ch.p_history[0];
  } 
  else {
    median_val = ch.p_history[2];
  }
  return median_val;
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
  char *strtokIndx = strtok(receivedChars, ",");

  for (int i = 0; i < 8; i++) {
    if (strtokIndx != NULL) {
      if (strcmp(strtokIndx, "off") == 0) {
        channels[i].isOff = true;
      } else {
        if (channels[i].isOff) {
          // Transitioning off → on: reset SMC state to avoid derivative spike
          channels[i].last_loop_time = millis();
          channels[i].prev_error = 0.0;
          channels[i].history_initialized = false;
        }
        channels[i].isOff = false;
        channels[i].targetPressure = atof(strtokIndx);
      }
    }
    strtokIndx = strtok(NULL, ",");
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
    channels[i].isOff              = true;   // All channels OFF until GUI sends targets

    // SMC internal state
    channels[i].prev_error     = 0.0;
    channels[i].last_loop_time = now;

    // Median filter state
    channels[i].p_history[0] = 0.0;
    channels[i].p_history[1] = 0.0;
    channels[i].p_history[2] = 0.0;
    channels[i].history_initialized = false;

    // Debug snapshot init
    channels[i].dbg_error = 0.0;
    channels[i].dbg_sat   = 0.0;
    channels[i].dbg_pulse = 0;

    // --- Assign Default SMC Gains (can be overridden per channel below) ---
    channels[i].lambda_smc        = DEF_LAMBDA;
    channels[i].phi               = DEF_PHI;
    channels[i].k_gain            = DEF_K_GAIN;
    channels[i].kd_gain           = DEF_KD_GAIN;
    channels[i].p_tolerance       = DEF_P_TOLERANCE;
    channels[i].passive_threshold = DEF_PASSIVE_THR;
    channels[i].vent_gain         = DEF_VENT_GAIN;
    channels[i].atmospheric_p     = DEF_ATMOSPHERIC_P;
  }

  // ==========================================================================
  // --- CUSTOM SMC TUNING FOR A SPECIFIC CHANNEL ---
  
    channels[0].p_tolerance       = 10.0;   // suction cup channel
    channels[1].p_tolerance       = 10.0;   // suction cup channel
  // ==========================================================================

  // Initialize Sensors
  if (!ads1.begin(0x48)) { Serial.println("ADS1 Failed"); while (1); }
  if (!ads2.begin(0x49)) { Serial.println("ADS2 Failed"); while (1); }

  ads1.setGain(GAIN_TWOTHIRDS);
  ads2.setGain(GAIN_TWOTHIRDS);

  Serial.println("<ARDUINO_READY_8CH>");
}

// ==========================================================================
//   MAIN LOOP
// ==========================================================================
void loop() {
  loopStartTime = millis();

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

    // --- OFF CHANNEL: skip sensor read and control entirely ---
    if (ch.isOff) {
      digitalWrite(ch.pinValvePos, LOW);
      digitalWrite(ch.pinValveNeg, LOW);
      ch.currentState = STATE_IDLE;
      ch.currentMode  = MODE_IDLE;
      ch.prev_error   = 0.0;
      ch.history_initialized = false;
      continue;
    }

    // A. Read Sensor
    ch.currentPressure = getCleanPressure(i);

    // Safety: sensor disconnected / wire break → close valves, skip control
    if (ch.currentPressure == -1000.0) {
      digitalWrite(ch.pinValvePos, LOW);
      digitalWrite(ch.pinValveNeg, LOW);
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
        unsigned long current_time = millis();
        float dt = (current_time - ch.last_loop_time) / 1000.0;

        // --- BRAIN SPEED LIMIT ---
        if (dt < 0.05) break;

        float error = ch.targetPressure - ch.currentPressure;
        bool should_sleep = false;

        // --- ADAPTIVE TOLERANCE LOGIC ---
        float currentTolerance = ch.p_tolerance; // Default
        if (ch.targetPressure <= -5.0) {
            currentTolerance = 20.0; // Wide tolerance for vacuum
        }

        // --- SUCTION CUP LOGIC ---
        if (ch.targetPressure <= -5.0) {
          if (ch.currentMode == MODE_IDLE) {
            // Resting: only wake up if we leak OUTSIDE the deadband
            if (abs(error) <= currentTolerance) should_sleep = true;
          } else {
            // Actively deflating: IGNORE the deadband,
            // keep pumping until we actually hit or cross the target
            if (ch.currentPressure <= ch.targetPressure) should_sleep = true;
          }
        }
        // --- STANDARD POSITIVE PRESSURE LOGIC ---
        else {
          if (abs(error) <= currentTolerance) should_sleep = true;
        }

        // --- EXECUTE SLEEP ---
        if (should_sleep) {
          digitalWrite(ch.pinValvePos, LOW);
          digitalWrite(ch.pinValveNeg, LOW);
          ch.currentMode = MODE_IDLE;
          ch.prev_error = error;
          ch.last_loop_time = current_time;
          break;
        }

        // --- Error derivative ---
        float error_dot = (error - ch.prev_error) / dt;

        // --- Sliding surface (with derivative gain) ---
        float s = (ch.lambda_smc * error) + (ch.kd_gain * error_dot);

        // --- Saturation function (clamp to [-1, +1]) ---
        float sat = s / ch.phi;
        if (sat >  1.0) sat =  1.0;
        if (sat < -1.0) sat = -1.0;

        // --- Pulse width ---
        unsigned long pulse_width = (unsigned long)(fabs(sat) * ch.k_gain);
        if (pulse_width > (unsigned long)MAX_PULSE) pulse_width = MAX_PULSE;

        // --- Record debug snapshot ---
        ch.dbg_error = error;
        ch.dbg_sat   = sat;
        ch.dbg_pulse = pulse_width;

        // ============================================================
        //  INFLATION  (sat > 0)
        // ============================================================
        if (sat > 0) {
          if (pulse_width > (unsigned long)MIN_PULSE) {
            if (pulse_width < (unsigned long)MIN_PUMP_PULSE) pulse_width = MIN_PUMP_PULSE;
            ch.currentMode = (fabs(sat) >= 0.85) ? MODE_COARSE_INF : MODE_FINE_INF;

            digitalWrite(ch.pinValveNeg, LOW);
            digitalWrite(ch.pinValvePos, HIGH);
            if (pulse_width >= (unsigned long)MIN_PUMP_PULSE) ch.reqPosPump = true;

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
                              (ch.currentPressure > ch.atmospheric_p);

            if (usePassive) {
              // --- Passive Vent (valve only, no pump) ---
              ch.currentMode = MODE_PASSIVE_DEF;
              digitalWrite(ch.pinValvePos, LOW);
              digitalWrite(ch.pinValveNeg, HIGH);
              ch.currentPulseDuration = (unsigned long)(pulse_width * (ch.vent_gain / ch.currentPressure));
              if (ch.currentPulseDuration > (unsigned long)MAX_PULSE) ch.currentPulseDuration = MAX_PULSE;
              ch.currentState   = STATE_DEFLATING_PASSIVE;
              ch.stateStartTime = millis();
            } else {
              // --- Active Vacuum (valve + pump) ---
              if (pulse_width < (unsigned long)MIN_PUMP_PULSE) pulse_width = MIN_PUMP_PULSE;
              ch.currentMode = (fabs(sat) >= 0.85) ? MODE_COARSE_DEF : MODE_FINE_DEF;
              digitalWrite(ch.pinValvePos, LOW);
              digitalWrite(ch.pinValveNeg, HIGH);
              if (pulse_width >= (unsigned long)MIN_PUMP_PULSE) ch.reqNegPump = true;

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
      case STATE_INFLATING: {
        if (ch.currentPulseDuration >= (unsigned long)MIN_PUMP_PULSE) ch.reqPosPump = true;

        // COARSE mode: keep valve open but monitor live pressure.
        // Once error shrinks below coarse exit threshold, close valve and rest
        // to prevent overshooting into a COARSE+/COARSE- limit cycle.
        if (ch.currentMode == MODE_COARSE_INF) {
          float liveError = ch.targetPressure - ch.currentPressure;
          if (liveError <= ch.p_tolerance) {
            // Close enough — stop and let pressure settle
            digitalWrite(ch.pinValvePos, LOW);
            ch.currentState   = STATE_RESTING;
            ch.stateStartTime = millis();
          }
          // Otherwise keep valve open, stay in this state
        }
        else if (millis() - ch.stateStartTime >= ch.currentPulseDuration) {
          digitalWrite(ch.pinValvePos, LOW);
          ch.currentState   = STATE_RESTING;
          ch.stateStartTime = millis();
        }
        break;
      }

      // ================================================================
      //  DEFLATING (Active) — Negative valve + vacuum pump
      // ================================================================
      case STATE_DEFLATING_ACTIVE: {
        if (ch.currentPulseDuration >= (unsigned long)MIN_PUMP_PULSE) ch.reqNegPump = true;

        // COARSE mode: keep valve open but monitor live pressure.
        if (ch.currentMode == MODE_COARSE_DEF) {
          float liveError = ch.targetPressure - ch.currentPressure;
          if (liveError >= -ch.p_tolerance) {
            // Close enough — stop and let pressure settle
            digitalWrite(ch.pinValveNeg, LOW);
            ch.currentState   = STATE_RESTING;
            ch.stateStartTime = millis();
          }
        }
        else if (millis() - ch.stateStartTime >= ch.currentPulseDuration) {
          digitalWrite(ch.pinValveNeg, LOW);
          ch.currentState   = STATE_RESTING;
          ch.stateStartTime = millis();
        }
        break;
      }

      // ================================================================
      //  DEFLATING (Passive) — Negative valve only, no pump
      // ================================================================
      case STATE_DEFLATING_PASSIVE:
        // No pump request — physics does the work
        if (millis() - ch.stateStartTime >= ch.currentPulseDuration) {
          ch.currentState = STATE_IDLE;
        }
        break;

      // ================================================================
      //  RESTING — All valves closed, wait for pressure to settle
      // ================================================================
      case STATE_RESTING: {
        digitalWrite(ch.pinValvePos, LOW);
        digitalWrite(ch.pinValveNeg, LOW);

        if (millis() - ch.stateStartTime >= (unsigned long)REST_TIME) {
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

    // --- DEBUG TELEMETRY (one line per active channel) ---
    // Format: DBG:ch,tgt,act,err,sat,pulse,mode,loopMs
    // GUI ignores these (only parses ACT: lines).
    // Open Serial Monitor to see them.
    if (DEBUG_TELEMETRY) {
      unsigned long loopMs = millis() - loopStartTime;
      for (int i = 0; i < 8; i++) {
        if (channels[i].isOff) continue;
        if (channels[i].currentPressure == -1000.0) continue;
        Serial.print("DBG:");
        Serial.print(i);                             Serial.print(",");
        Serial.print(channels[i].targetPressure, 1); Serial.print(",");
        Serial.print(channels[i].currentPressure, 2);Serial.print(",");
        Serial.print(channels[i].dbg_error, 2);      Serial.print(",");
        Serial.print(channels[i].dbg_sat, 3);        Serial.print(",");
        Serial.print(channels[i].dbg_pulse);          Serial.print(",");
        Serial.print(channels[i].currentMode);        Serial.print(",");
        Serial.println(loopMs);
      }
    }
  }
}
