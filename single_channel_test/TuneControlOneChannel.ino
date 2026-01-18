#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// --- CONFIGURATION ---
float TARGET_P = 10.0;          // Default Target (Can be changed via Serial)
const float E = 0.5;            // Fine Error Threshold

// --- TUNING (HYBRID + ADAPTIVE) ---
const float COARSE_THRESHOLD = 5.0; // Threshold for Continuous Mode
const float COARSE_STOP = 4.7;   // when the coarse fill stops. smaller => undershoot, bigger = overshoot.
const int REST_TIME = 500;  

// 1. INFLATION TUNING
const float Kp_Inflate_Base = 10.0;  // ACTIVE PUMP GAIN (Pushing against pressure)
const float Kp_Inflate_Slope = 0.8;  
const float Kp_Inflate_Passive = 100.0; // NATURAL GAIN (Vacuum sucking air in). Usually needs to be higher.

// 2. DEFLATION TUNING
const float Kp_Deflate_Base = 5.0;   // ACTIVE PUMP GAIN (Vacuum pump pulling air out)
const float Kp_Deflate_Slope = 0.0; 
const float Kp_Deflate_Passive = 100.0; // NATURAL GAIN (Pressure pushing air out). Gentler than pump, so Kp is higher.

// PUMP & VALVE TUNING 
const int MIN_PULSE_PUMP = 30;   // Hard limit for PUMP (Motors need time to spin up)
const int MIN_PULSE_VALVE = 10;  // Hard limit for VALVE ONLY (Solenoids are fast!)
const int MAX_PULSE = 2000;      // Safety cap

// LOG STATUS SPEED
const int STATUS_SPEED = 100;  // print status log every x milliseconds

// --- SENSOR SETUP ---
const float SENSOR_OFFSET = -0.85;  // The pressure reading of the sensor when the output tube is open
const float ADS_BIT_VOLTAGE = 0.0001875; 
const float V_OFFSET = 0.5;      
const float V_FULLSCALE = 4.5;   
const float P_MIN_KPA = -100.0;     
const float P_MAX_KPA = 100.0;    
const float V_SPAN = V_FULLSCALE - V_OFFSET; 
const float P_SPAN = P_MAX_KPA - P_MIN_KPA; 

// --- PINS ---
const int positivePump = 12;       
const int negativePump = 13;       
const int positiveValveCH1 = 33;   
const int negativeValveCH1 = 39;
const int sensorCH = 2;

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
unsigned long lastStatusPrint = 0; // For the 1Hz heartbeat 

// --- HELPER: Read Pressure ---
float readSensor(Adafruit_ADS1115& adsModule, int channel) {
  int16_t raw = adsModule.readADC_SingleEnded(channel);
  float voltage = raw * ADS_BIT_VOLTAGE;   
  if (voltage < 0.2) return -1000.0; 
  if (voltage < V_OFFSET) voltage = V_OFFSET;
  return (P_MIN_KPA + ( (voltage - V_OFFSET) / V_SPAN ) * P_SPAN) - SENSOR_OFFSET;
}

// --- HELPER: Get State Name as String ---
String getStateName(SystemState s) {
  switch(s) {
    case STATE_IDLE: return "IDLE";
    case STATE_COARSE_INFLATE: return "COARSE_INF";
    case STATE_FINE_INFLATE: return "FINE_INF";
    case STATE_COARSE_DEFLATE: return "COARSE_DEF";
    case STATE_FINE_DEFLATE: return "FINE_DEF";
    case STATE_RESTING: return "RESTING";
    default: return "UNKNOWN";
  }
}

// --- HELPER: Calculate Adaptive Kp ---
// UPDATED: Selects between MIN_PULSE_PUMP and MIN_PULSE_VALVE
unsigned long getAdaptivePulse(float error, float currentP, bool isInflating) {
  float duration = 0;
  float absP = abs(currentP);
  float safeP = (absP < 0.5) ? 0.5 : absP; 
  
  // Default to the slower PUMP limit
  int currentMinLimit = MIN_PULSE_PUMP;

  if (isInflating) {
    if (currentP < 0) {
      // NATURAL INFLATION (Valve Only) -> Use Fast Valve Limit
      duration = (abs(error) / safeP) * Kp_Inflate_Passive;
      currentMinLimit = MIN_PULSE_VALVE;
    } else {
      // ACTIVE INFLATION (Pump + Valve) -> Use Slow Pump Limit
      float kp = Kp_Inflate_Base + (Kp_Inflate_Slope * currentP);
      duration = abs(error) * kp;
    }
  } else { // Deflating
    if (currentP > 0) {
      // NATURAL DEFLATION (Valve Only) -> Use Fast Valve Limit
      duration = (abs(error) / safeP) * Kp_Deflate_Passive;
      currentMinLimit = MIN_PULSE_VALVE;
    } else {
      // ACTIVE DEFLATION (Pump + Valve) -> Use Slow Pump Limit
      float kp = Kp_Deflate_Base + (Kp_Deflate_Slope * currentP);
      duration = abs(error) * kp;
    }
  }

  // Apply the specific limit for this mode
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

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50); // Fast timeout for parsing floats

  pinMode(positivePump, OUTPUT); digitalWrite(positivePump, LOW); 
  pinMode(negativePump, OUTPUT); digitalWrite(negativePump, LOW); 
  pinMode(positiveValveCH1, OUTPUT); digitalWrite(positiveValveCH1, LOW); 
  pinMode(negativeValveCH1, OUTPUT); digitalWrite(negativeValveCH1, LOW); 

  if (!ads.begin()) { Serial.println("ADS Failed"); while (1); }
  ads.setGain(GAIN_TWOTHIRDS); 
  Serial.println("--- READY v3.2 (Dual Pulse Limits) ---");
}

void loop() {
  // 1. READ SENSOR
  float P_Actual = readSensor(ads, sensorCH);
  if (P_Actual == -1000.0) { stopAll(); Serial.println("SENSOR ERROR"); return; }

  // 2. CHECK SERIAL INPUT (Update Target)
  if (Serial.available() > 0) {
    float newTarget = Serial.parseFloat(); // Reads the float
    while(Serial.available()) Serial.read(); 
    
    if (newTarget >= P_MIN_KPA && newTarget <= P_MAX_KPA) {
      TARGET_P = newTarget;
      Serial.print(">>> NEW TARGET RECEIVED: "); Serial.print(TARGET_P); Serial.println(" kPa <<<");
      stopAll();
      currentState = STATE_IDLE; 
    }
  }

  // 3. HEARTBEAT PRINT (Once per second)
  if (millis() - lastStatusPrint >= STATUS_SPEED) {
    lastStatusPrint = millis();
    Serial.print("[STATUS] State: "); 
    Serial.print(getStateName(currentState));
    Serial.print(" | P: "); Serial.print(P_Actual, 2);
    Serial.print(" | Tgt: "); Serial.print(TARGET_P, 2);
    Serial.print(" | Err: "); Serial.println(TARGET_P - P_Actual, 2);
  }

  // 4. STATE MACHINE
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
             Serial.print("COARSE INFLATE");
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
                Serial.print("FINE INFLATE");
             } else {
                Serial.print("FINE INFLATE NATURAL");  // Pump remains LOW (Passive Intake)
             }
          }

        } else { 
          // >>> NEED DEFLATE (Target < Current) <<<
          
          if (abs(error) > COARSE_THRESHOLD) {
             // 3. Coarse Deflate (Always Pump)
             currentState = STATE_COARSE_DEFLATE;
             Serial.print("COARSE DEFLATE");
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
                Serial.print("FINE DEFLATE");
             } else {
                Serial.print("FINE DEFLATE NATURAL");  // Pump remains LOW (Passive Venting)
             }
          }
        }
        Serial.print(" | PULSE: "); Serial.println(currentPulseDuration);
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