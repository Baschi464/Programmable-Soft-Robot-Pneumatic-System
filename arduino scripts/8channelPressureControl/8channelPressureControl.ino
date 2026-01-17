#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ==========================================================================
//   8-CHANNEL PNEUMATIC CONTROLLER (SHARED PUMPS, INDEPENDENT CHANNELS)
// ==========================================================================

// --- CONFIGURATION & TUNING DEFAULTS ---
const float DEFAULT_TARGET = 0.0;
const float E = 0.5;                // Fine Error Threshold
const float COARSE_THRESHOLD = 5.0; // Threshold for Continuous Mode
const float COARSE_STOP = 4.7;      // Offset to stop coarse fill early
const int REST_TIME = 500;          // Time to settle between pulses

// --- GAIN DEFAULTS (Applied to all channels initially) ---
// 1. INFLATION
const float DEF_Kp_Inflate_Base = 10.0;     
const float DEF_Kp_Inflate_Slope = 0.8;      
const float DEF_Kp_Inflate_Passive = 100.0; 

// 2. DEFLATION
const float DEF_Kp_Deflate_Base = 5.0;      
const float DEF_Kp_Deflate_Slope = 0.0;      
const float DEF_Kp_Deflate_Passive = 100.0; 

// --- PULSE LIMITS ---  
const int MIN_PULSE_PUMP = 30;   // Hard limit for PUMP 
const int MIN_PULSE_VALVE = 20;  // Hard limit for VALVE ONLY (the loop should take up to 20ms so thats another limit, it is possible to go faster by increasing I2C bus clock speed but that risks noise and can cause it to freeze)
const int MAX_PULSE = 2000;      // Safety cap

// --- SENSOR SETTINGS ---
const float ADS_BIT_VOLTAGE = 0.0001875; 
const float V_OFFSET = 0.5;       
const float V_FULLSCALE = 4.5;    
const float P_MIN_KPA = -100.0;      
const float P_MAX_KPA = 100.0;     
const float V_SPAN = V_FULLSCALE - V_OFFSET; 
const float P_SPAN = P_MAX_KPA - P_MIN_KPA; 
// --- INDIVIDUAL SENSOR OFFSETS (Calibration) ---
float SENSOR_OFFSETS[8] = { -0.91, -0.68, -0.65, 0.0, 0.0, 0.0, 0.0, 0.0 };

// --- PINS (USER TO CONFIGURE) ---
// Shared Pumps
const int PIN_PUMP_POS = 12;      
const int PIN_PUMP_NEG = 13;      

// Valve Arrays (Index 0 = Channel 1, Index 1 = Channel 2, etc.)
// UPDATE THESE PIN NUMBERS TO MATCH YOUR WIRING!
const int VALVES_POS[8] = { 2, 40, 40, 40, 40, 40, 40, 40 }; 
const int VALVES_NEG[8] = { 35, 40, 40, 40, 40, 40, 40, 40 };

// --- STATE ENUM ---
enum SystemState {
  STATE_IDLE,
  STATE_COARSE_INFLATE,
  STATE_FINE_INFLATE,
  STATE_COARSE_DEFLATE,
  STATE_FINE_DEFLATE,
  STATE_RESTING
};

// --- CHANNEL OBJECT STRUCT ---
// Holds all variables for a single channel so they operate independently
struct ChannelControl {
  // Config
  int pinValvePos;
  int pinValveNeg;
  
  // Dynamic Variables
  float targetPressure;
  float currentPressure;
  SystemState currentState;
  unsigned long stateStartTime;
  unsigned long currentPulseDuration;
  
  // Pump Requests (Flags for the Pump Manager)
  bool reqPosPump;
  bool reqNegPump;

  // Independent Gains (Copied from defaults, can be tweaked per channel)
  float Kp_Inf_Base;
  float Kp_Inf_Slope;
  float Kp_Inf_Pass;
  float Kp_Def_Base;
  float Kp_Def_Slope;
  float Kp_Def_Pass;
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
const long TELEMETRY_INTERVAL = 100; // 10Hz updates to Python

// ==========================================================================
//   HELPER FUNCTIONS
// ==========================================================================

// --- Read Pressure (Selects correct ADS module) ---
float readPressure(int chIndex) {
  int16_t raw;
  float voltage;
  
  // Map index 0-3 to ads1, 4-7 to ads2
  if (chIndex < 4) {
    raw = ads1.readADC_SingleEnded(chIndex); 
  } else {
    raw = ads2.readADC_SingleEnded(chIndex - 4);
  }

  voltage = raw * ADS_BIT_VOLTAGE;   
  if (voltage < 0.2) return -1000.0; // Error or Disconnected
  if (voltage < V_OFFSET) voltage = V_OFFSET;
  
  float kpa = (P_MIN_KPA + ( (voltage - V_OFFSET) / V_SPAN ) * P_SPAN) - SENSOR_OFFSETS[chIndex];
  if (kpa < -90.0) return -1000.0; // Error or Disconnected 
  return kpa;
}

// --- Adaptive Pulse Calculation (Per Channel) ---
unsigned long getAdaptivePulse(ChannelControl &ch, float error, bool isInflating) {
  float duration = 0;
  float absP = abs(ch.currentPressure);
  float safeP = (absP < 0.5) ? 0.5 : absP; 
  int currentMinLimit = MIN_PULSE_PUMP;

  if (isInflating) {
    if (ch.currentPressure < 0) {
      // NATURAL INFLATION (Valve Only)
      duration = (abs(error) / safeP) * ch.Kp_Inf_Pass;
      currentMinLimit = MIN_PULSE_VALVE;
    } else {
      // ACTIVE INFLATION (Pump + Valve)
      float kp = ch.Kp_Inf_Base + (ch.Kp_Inf_Slope * ch.currentPressure);
      duration = abs(error) * kp;
    }
  } else { // Deflating
    if (ch.currentPressure > 0) {
      // NATURAL DEFLATION (Valve Only)
      duration = (abs(error) / safeP) * ch.Kp_Def_Pass;
      currentMinLimit = MIN_PULSE_VALVE;
    } else {
      // ACTIVE DEFLATION (Pump + Valve)
      float kp = ch.Kp_Def_Base + (ch.Kp_Def_Slope * ch.currentPressure);
      duration = abs(error) * kp;
    }
  }

  if (duration < currentMinLimit) duration = currentMinLimit;
  if (duration > MAX_PULSE) duration = MAX_PULSE; 
  
  return (unsigned long)duration;
}

// --- Serial Parsing (Matches Simulation Format) ---
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
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {
  char * strtokIndx; 
  strtokIndx = strtok(receivedChars, ","); 
  
  if(strtokIndx != NULL) {
    channels[0].targetPressure = atof(strtokIndx);
  }
  
  for(int i=1; i<8; i++) {
    strtokIndx = strtok(NULL, ","); 
    if (strtokIndx != NULL) {
      channels[i].targetPressure = atof(strtokIndx);
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
  Wire.setClock(100000); // Set I2C to 100kHz (Fast Mode would be 400kHz but we risk noise and I2C bus freezing)

  // Pump Pins
  pinMode(PIN_PUMP_POS, OUTPUT); digitalWrite(PIN_PUMP_POS, LOW);
  pinMode(PIN_PUMP_NEG, OUTPUT); digitalWrite(PIN_PUMP_NEG, LOW);

  // Initialize Channels
  for (int i = 0; i < 8; i++) {
    channels[i].pinValvePos = VALVES_POS[i];
    channels[i].pinValveNeg = VALVES_NEG[i];
    pinMode(channels[i].pinValvePos, OUTPUT); digitalWrite(channels[i].pinValvePos, LOW);
    pinMode(channels[i].pinValveNeg, OUTPUT); digitalWrite(channels[i].pinValveNeg, LOW);

    channels[i].currentState = STATE_IDLE;
    channels[i].targetPressure = DEFAULT_TARGET;
    channels[i].reqPosPump = false;
    channels[i].reqNegPump = false;
    
    // Assign Default Gains (Can be customized per index here if needed)
    channels[i].Kp_Inf_Base = DEF_Kp_Inflate_Base;
    channels[i].Kp_Inf_Slope = DEF_Kp_Inflate_Slope;
    channels[i].Kp_Inf_Pass = DEF_Kp_Inflate_Passive;
    channels[i].Kp_Def_Base = DEF_Kp_Deflate_Base;
    channels[i].Kp_Def_Slope = DEF_Kp_Deflate_Slope;
    channels[i].Kp_Def_Pass = DEF_Kp_Deflate_Passive;
  }

  // ==========================================================================
  // // --- EXAMPLE OF CUSTOM TUNING FOR CHANNEL 4 (Index 3) ---
  // // Example: Channel 4 is connected to a smaller volume, so we reduce gains to prevent overshoot.
  
  // // 1. Reduce Active Inflation Gain (Pump + Valve)
  // channels[3].Kp_Inf_Base = 5.0;   // Default was 10.0
  
  // // 2. Increase Passive Inflation Gain (Vacuum only)
  // channels[3].Kp_Inf_Pass = 150.0; // Default was 100.0

  // // 3. You can also change Deflation gains specifically for this channel
  // channels[3].Kp_Def_Base = 2.5;   // Default was 5.0
  
  // // Note: All other channels (0, 1, 2, 4, 5, 6, 7) still use the defaults.
  // ==========================================================================

  // Initialize Sensors
  // ADS1: Addr 0x48, ADS2: Addr 0x49
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
  
  // 1. COMMUNICATE (Receive Targets)
  recvWithStartEndMarkers();
  if (newData == true) {
    parseData();
    newData = false;
    // Note: We do NOT stop all channels on new data; we just update the targets dynamically.
  }

  // 2. CONTROL LOOP (Iterate 8 Channels)
  // We reset pump requests before checking channels.
  // The Pump Manager at the end will decide if pumps turn on.
  bool anyChannelNeedsPosPump = false;
  bool anyChannelNeedsNegPump = false;

  for (int i = 0; i < 8; i++) {
    ChannelControl &ch = channels[i]; // Reference for easier code
    
    // A. Read Sensor
    ch.currentPressure = readPressure(i);

    // --- SAFETY CHECK: SENSOR DISCONNECT ---
    if (ch.currentPressure == -1000.0) {
      // Force everything OFF for this channel
      digitalWrite(ch.pinValvePos, LOW);
      digitalWrite(ch.pinValveNeg, LOW);
      ch.currentState = STATE_IDLE; // Reset state
      continue; // Skip the rest of the loop for this channel
    }
    // Reset pump flags for this channel for this cycle
    ch.reqPosPump = false;
    ch.reqNegPump = false;

    // B. Calculate Error
    float error = ch.targetPressure - ch.currentPressure;

    // C. State Machine
    switch (ch.currentState) {
    
      case STATE_IDLE:
        if (abs(error) > E) {
          if (error > 0) { 
            // >>> NEED INFLATE <<<
            if (error > COARSE_THRESHOLD) {
               // 1. Coarse Inflate
               ch.currentState = STATE_COARSE_INFLATE;
               digitalWrite(ch.pinValveNeg, LOW);
               digitalWrite(ch.pinValvePos, HIGH);
               ch.reqPosPump = true; 
            } else {
               // 2. Fine Inflate
               ch.currentPulseDuration = getAdaptivePulse(ch, error, true);
               ch.currentState = STATE_FINE_INFLATE;
               ch.stateStartTime = millis();
               
               digitalWrite(ch.pinValveNeg, LOW);
               digitalWrite(ch.pinValvePos, HIGH);
               
               // Natural vs Active Logic
               if (ch.currentPressure >= 0.0) {
                 ch.reqPosPump = true; // Active
               } 
               // else: Passive (Vacuum sucks air in), pump stays false
            }

          } else { 
            // >>> NEED DEFLATE <<<
            if (abs(error) > COARSE_THRESHOLD) {
               // 3. Coarse Deflate
               ch.currentState = STATE_COARSE_DEFLATE;
               digitalWrite(ch.pinValvePos, LOW);
               digitalWrite(ch.pinValveNeg, HIGH);
               ch.reqNegPump = true; 
            } else {
               // 4. Fine Deflate
               ch.currentPulseDuration = getAdaptivePulse(ch, error, false);
               ch.currentState = STATE_FINE_DEFLATE;
               ch.stateStartTime = millis();
               
               digitalWrite(ch.pinValvePos, LOW);
               digitalWrite(ch.pinValveNeg, HIGH);
               
               // Natural vs Active Logic
               if (ch.currentPressure <= 0.0) {
                 ch.reqNegPump = true; // Active
               }
               // else: Passive (Pressure pushes air out), pump stays false
            }
          }
        }
        break;

      case STATE_COARSE_INFLATE:
        // Logic: Keep Valve Open, Request Pump
        digitalWrite(ch.pinValvePos, HIGH);
        ch.reqPosPump = true;

        if (ch.currentPressure >= (ch.targetPressure + COARSE_STOP)) {
          // Stop
          digitalWrite(ch.pinValvePos, LOW);
          ch.reqPosPump = false;
          ch.currentState = STATE_RESTING;
          ch.stateStartTime = millis();
        }
        break;

      case STATE_FINE_INFLATE:
        // Logic: Valve Open. Pump requested only if Active mode was decided in IDLE.
        // We need to re-evaluate pump request based on pressure? 
        // No, keep consistent with the pulse start decision to avoid chatter.
        // However, since we cleared flags at start of loop, we re-assert logic:
        if (ch.currentPressure >= 0.0) ch.reqPosPump = true; 
        
        // Timeout check
        if (millis() - ch.stateStartTime >= ch.currentPulseDuration) {
          digitalWrite(ch.pinValvePos, LOW);
          ch.currentState = STATE_RESTING;
          ch.stateStartTime = millis();
        }
        break;

      case STATE_COARSE_DEFLATE:
        digitalWrite(ch.pinValveNeg, HIGH);
        ch.reqNegPump = true;

        if (ch.currentPressure <= (ch.targetPressure - COARSE_STOP)) {
          digitalWrite(ch.pinValveNeg, LOW);
          ch.reqNegPump = false;
          ch.currentState = STATE_RESTING;
          ch.stateStartTime = millis();
        }
        break;

      case STATE_FINE_DEFLATE:
        // Re-assert active pump logic
        if (ch.currentPressure <= 0.0) ch.reqNegPump = true;

        if (millis() - ch.stateStartTime >= ch.currentPulseDuration) {
          digitalWrite(ch.pinValveNeg, LOW);
          ch.currentState = STATE_RESTING;
          ch.stateStartTime = millis();
        }
        break;

      case STATE_RESTING:
        // Both Valves Closed, No Pump Request
        digitalWrite(ch.pinValvePos, LOW);
        digitalWrite(ch.pinValveNeg, LOW);
        
        if (millis() - ch.stateStartTime >= REST_TIME) {
          ch.currentState = STATE_IDLE;
        }
        break;
    }

    // Accumulate Pump Requests
    if (ch.reqPosPump) anyChannelNeedsPosPump = true;
    if (ch.reqNegPump) anyChannelNeedsNegPump = true;
  }

  // 3. PUMP MANAGER
  // Since user confirmed power supply can handle both:
  digitalWrite(PIN_PUMP_POS, anyChannelNeedsPosPump ? HIGH : LOW);
  digitalWrite(PIN_PUMP_NEG, anyChannelNeedsNegPump ? HIGH : LOW);

  // 4. TELEMETRY (Send to Python)
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
