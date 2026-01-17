#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ==========================================================================
//   1-CHANNEL CONTROL + 8-CHANNEL COMMUNICATION PROTOCOL
// ==========================================================================

// --- CONFIGURATION ---
float TARGET_P = 0.0;           // Default Target
const float E = 0.5;            // Fine Error Threshold

// --- TUNING (HYBRID + ADAPTIVE) ---
const float COARSE_THRESHOLD = 5.0; // Threshold for Continuous Mode
const float COARSE_STOP = 4.7;      // Coarse fill stop offset
const int REST_TIME = 500;   

// 1. INFLATION TUNING
const float Kp_Inflate_Base = 10.0;      // ACTIVE PUMP GAIN
const float Kp_Inflate_Slope = 0.8;  
const float Kp_Inflate_Passive = 100.0;  // NATURAL GAIN

// 2. DEFLATION TUNING
const float Kp_Deflate_Base = 5.0;       // ACTIVE PUMP GAIN
const float Kp_Deflate_Slope = 0.0; 
const float Kp_Deflate_Passive = 100.0;  // NATURAL GAIN

// PUMP & VALVE TUNING 
const int MIN_PULSE_PUMP = 30;   // Hard limit for PUMP
const int MIN_PULSE_VALVE = 10;  // Hard limit for VALVE ONLY
const int MAX_PULSE = 2000;      // Safety cap

// --- SENSOR SETUP ---
const float SENSOR_OFFSET = -0.85;  
const float ADS_BIT_VOLTAGE = 0.0001875; 
const float V_OFFSET = 0.5;      
const float V_FULLSCALE = 4.5;   
const float P_MIN_KPA = -100.0;     
const float P_MAX_KPA = 100.0;    
const float V_SPAN = V_FULLSCALE - V_OFFSET; 
const float P_SPAN = P_MAX_KPA - P_MIN_KPA; 

// --- PINS (From 1-Channel Code) ---
const int positivePump = 12;       
const int negativePump = 13;       
const int positiveValveCH1 = 2;   
const int negativeValveCH1 = 35;
const int sensorCH = 0;

// --- STATE MACHINE ---
enum SystemState {
  STATE_IDLE,
  STATE_COARSE_INFLATE,
  STATE_FINE_INFLATE,
  STATE_COARSE_DEFLATE,
  STATE_FINE_DEFLATE,
  STATE_RESTING
};
SystemState currentState = STATE_IDLE;

// --- VARIABLES ---
Adafruit_ADS1115 ads; 
unsigned long stateStartTime = 0;   
unsigned long currentPulseDuration = 0; 

// --- COMMUNICATION VARIABLES ---
const byte numChars = 128;
char receivedChars[numChars];
boolean newData = false;
unsigned long lastTelemetryTime = 0;
const long TELEMETRY_INTERVAL = 100; // 10Hz updates to Python (matches 8ch code)

// ==========================================================================
//   HELPER FUNCTIONS
// ==========================================================================

// --- Read Pressure ---
float readSensor(Adafruit_ADS1115& adsModule, int channel) {
  int16_t raw = adsModule.readADC_SingleEnded(channel);
  float voltage = raw * ADS_BIT_VOLTAGE;   
  if (voltage < 0.2) return -1000.0; 
  if (voltage < V_OFFSET) voltage = V_OFFSET;
  return (P_MIN_KPA + ( (voltage - V_OFFSET) / V_SPAN ) * P_SPAN) - SENSOR_OFFSET;
}

// --- Adaptive Pulse Calculation ---
unsigned long getAdaptivePulse(float error, float currentP, bool isInflating) {
  float duration = 0;
  float absP = abs(currentP);
  float safeP = (absP < 0.5) ? 0.5 : absP; 
  int currentMinLimit = MIN_PULSE_PUMP;

  if (isInflating) {
    if (currentP < 0) {
      // NATURAL INFLATION (Valve Only)
      duration = (abs(error) / safeP) * Kp_Inflate_Passive;
      currentMinLimit = MIN_PULSE_VALVE;
    } else {
      // ACTIVE INFLATION (Pump + Valve)
      float kp = Kp_Inflate_Base + (Kp_Inflate_Slope * currentP);
      duration = abs(error) * kp;
    }
  } else { // Deflating
    if (currentP > 0) {
      // NATURAL DEFLATION (Valve Only)
      duration = (abs(error) / safeP) * Kp_Deflate_Passive;
      currentMinLimit = MIN_PULSE_VALVE;
    } else {
      // ACTIVE DEFLATION (Pump + Valve)
      float kp = Kp_Deflate_Base + (Kp_Deflate_Slope * currentP);
      duration = abs(error) * kp;
    }
  }

  if (duration < currentMinLimit) duration = currentMinLimit;
  if (duration > MAX_PULSE) duration = MAX_PULSE; 
  
  return (unsigned long)duration;
}

void stopAll() {
  digitalWrite(positivePump, LOW); 
  digitalWrite(negativePump, LOW); 
  digitalWrite(positiveValveCH1, LOW); 
  digitalWrite(negativeValveCH1, LOW);
}

// --- SERIAL RECEIVE (Matches 8-Channel Protocol) ---
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

// --- PARSE DATA ---
// We parse the list, but only use the FIRST value for our single channel.
void parseData() {
  char * strtokIndx; 
  strtokIndx = strtok(receivedChars, ","); // get the first part
  
  if(strtokIndx != NULL) {
    float newTarget = atof(strtokIndx); // This is Channel 1
    
    // Only update if target changed significantly or if we want to enforce logic
    if (newTarget != TARGET_P) {
        if (newTarget >= P_MIN_KPA && newTarget <= P_MAX_KPA) {
          TARGET_P = newTarget;
          // Note: In 8ch code we don't force stop on new target, 
          // but in 1ch code we did. Keeping 8ch behavior (smooth transition)
          // to prevent jerky movement during slider dragging in GUI.
        }
    }
  }
  // We ignore the rest of the tokens since we only have 1 channel
}

// ==========================================================================
//   SETUP
// ==========================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); // Match 8ch timeout

  pinMode(positivePump, OUTPUT); digitalWrite(positivePump, LOW); 
  pinMode(negativePump, OUTPUT); digitalWrite(negativePump, LOW); 
  pinMode(positiveValveCH1, OUTPUT); digitalWrite(positiveValveCH1, LOW); 
  pinMode(negativeValveCH1, OUTPUT); digitalWrite(negativeValveCH1, LOW); 

  if (!ads.begin()) { Serial.println("ADS Failed"); while (1); }
  ads.setGain(GAIN_TWOTHIRDS); 
  
  // Handshake for Python GUI
  Serial.println("<ARDUINO_READY_8CH>");
}

// ==========================================================================
//   MAIN LOOP
// ==========================================================================
void loop() {
  // 1. READ SENSOR
  float P_Actual = readSensor(ads, sensorCH);
  if (P_Actual == -1000.0) { stopAll(); } // We handle the error reporting in Telemetry section

  // 2. COMMUNICATE (Receive Targets)
  recvWithStartEndMarkers();
  if (newData == true) {
    parseData();
    newData = false;
  }

  // 3. TELEMETRY (Send to Python in 8-channel format)
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = millis();
    
    // Format: ACT:v1,v2,v3,v4,v5,v6,v7,v8
    Serial.print("ACT:");
    
    // Channel 1 (Real)
    if (P_Actual == -1000.0) Serial.print("ERR");
    else Serial.print(P_Actual, 2);
    
    // Channels 2-8 (Fake/Errors)
    Serial.print(",ERR,ERR,ERR,ERR,ERR,ERR,ERR");
    Serial.println();
  }

  // 4. CONTROL LOGIC (Original 1-Channel State Machine)
  if (P_Actual == -1000.0) return; // Skip control if sensor error

  float error = TARGET_P - P_Actual;

  switch (currentState) {
    
    case STATE_IDLE:
      if (abs(error) > E) {
        
        // --- DECISION: INFLATE OR DEFLATE? ---
        if (error > 0) { 
          // >>> NEED INFLATE (Target > Current) <<<
          
          if (error > COARSE_THRESHOLD) {
             // 1. Coarse Inflate (Always Pump)
             currentState = STATE_COARSE_INFLATE;
             digitalWrite(negativeValveCH1, LOW);
             digitalWrite(positiveValveCH1, HIGH);
             delay(20);
             digitalWrite(positivePump, HIGH);
          } else {
             // 2. Fine Inflate (Smart)
             currentPulseDuration = getAdaptivePulse(error, P_Actual, true);
             currentState = STATE_FINE_INFLATE;
             stateStartTime = millis();
             
             digitalWrite(negativeValveCH1, LOW);
             digitalWrite(positiveValveCH1, HIGH);
             delay(10);

             // STRATEGY: If we are in vacuum (P < 0), nature will fill it for us.
             if (P_Actual >= 0.0) {
                digitalWrite(positivePump, HIGH); 
             } 
          }

        } else { 
          // >>> NEED DEFLATE (Target < Current) <<<
          
          if (abs(error) > COARSE_THRESHOLD) {
             // 3. Coarse Deflate (Always Pump)
             currentState = STATE_COARSE_DEFLATE;
             digitalWrite(positiveValveCH1, LOW);
             digitalWrite(negativeValveCH1, HIGH);
             delay(20);
             digitalWrite(negativePump, HIGH);
          } else {
             // 4. Fine Deflate (Smart)
             currentPulseDuration = getAdaptivePulse(error, P_Actual, false);
             currentState = STATE_FINE_DEFLATE;
             stateStartTime = millis();
             
             digitalWrite(positiveValveCH1, LOW);
             digitalWrite(negativeValveCH1, HIGH);
             delay(10);

             // STRATEGY: If we have positive pressure (P > 0), nature will vent it for us.
             if (P_Actual <= 0.0) {
                digitalWrite(negativePump, HIGH);
             } 
          }
        }
      }
      break;

    case STATE_COARSE_INFLATE:
      if (P_Actual >= (TARGET_P + COARSE_STOP)) {
        stopAll();
        currentState = STATE_RESTING;
        stateStartTime = millis();
      }
      break;

    case STATE_FINE_INFLATE:
      if (millis() - stateStartTime >= currentPulseDuration) {
        stopAll();
        currentState = STATE_RESTING;
        stateStartTime = millis();
      }
      break;

    case STATE_COARSE_DEFLATE:
      if (P_Actual <= (TARGET_P - COARSE_STOP)) {
        stopAll();
        currentState = STATE_RESTING;
        stateStartTime = millis();
      }
      break;

    case STATE_FINE_DEFLATE:
      if (millis() - stateStartTime >= currentPulseDuration) {
        stopAll();
        currentState = STATE_RESTING;
        stateStartTime = millis();
      }
      break;

    case STATE_RESTING:
      if (millis() - stateStartTime >= REST_TIME) {
        currentState = STATE_IDLE;
      }
      break;
  }
}