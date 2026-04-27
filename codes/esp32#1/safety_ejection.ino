// ═══════════════════════════════════════════════
// ESP32 #2 - SAFETY & EJECTION
// safety_ejection.ino
// ═══════════════════════════════════════════════

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

// ── PIN DEFINITIONS ───────────────────────────
// Inductive sensors
#define IND_SENSOR_1    34
#define IND_SENSOR_2    35
#define IND_SENSOR_3    36

// Thermal sensor
Adafruit_MLX90614 mlx;

// Gas sensor
#define GAS_SENSOR      32
#define GAS_THRESHOLD   400

// Air Jets (via Relay)
#define JET_1           25  // PET
#define JET_2           26  // HDPE/PP
#define JET_3           27  // Paper
#define JET_4           14  // Metal
#define JET_5           12  // Glass
#define JET_6           13  // HAZARD ⚠️

// Safety
#define ESTOP_BTN       4
#define ESTOP_RELAY     5
#define BUZZER          18
#define LED_RED         19
#define LED_GREEN       23

// ── STATE ─────────────────────────────────────
bool emergency_stop_active = false;
bool metal_detected = false;
bool battery_suspected = false;

// Jet timing
struct JetCommand {
    uint8_t jet_id;
    uint32_t fire_time;
    uint16_t duration_ms;
    bool alarm;
};

#define MAX_PENDING_JETS 10
JetCommand pending_jets[MAX_PENDING_JETS];
uint8_t pending_count = 0;

// ═════════════════════════════════════════════
void setup() {
    Serial.begin(115200);  // To Jetson
    
    // Pins
    pinMode(IND_SENSOR_1, INPUT_PULLUP);
    pinMode(IND_SENSOR_2, INPUT_PULLUP);
    pinMode(IND_SENSOR_3, INPUT_PULLUP);
    
    pinMode(JET_1, OUTPUT); digitalWrite(JET_1, LOW);
    pinMode(JET_2, OUTPUT); digitalWrite(JET_2, LOW);
    pinMode(JET_3, OUTPUT); digitalWrite(JET_3, LOW);
    pinMode(JET_4, OUTPUT); digitalWrite(JET_4, LOW);
    pinMode(JET_5, OUTPUT); digitalWrite(JET_5, LOW);
    pinMode(JET_6, OUTPUT); digitalWrite(JET_6, LOW);
    
    pinMode(ESTOP_BTN,   INPUT_PULLUP);
    pinMode(ESTOP_RELAY, OUTPUT);
    pinMode(BUZZER,      OUTPUT);
    pinMode(LED_RED,     OUTPUT);
    pinMode(LED_GREEN,   OUTPUT);
    
    // Start safe
    digitalWrite(ESTOP_RELAY, HIGH);  // Relay NC = power ON
    digitalWrite(LED_GREEN, HIGH);
    
    // Thermal sensor
    mlx.begin();
    
    // Interrupt for E-Stop
    attachInterrupt(
        digitalPinToInterrupt(ESTOP_BTN),
        IRAM_ATTR emergency_stop_ISR,
        FALLING
    );
    
    Serial.println("{\"status\":\"safety_ready\"}");
}

// ═════════════════════════════════════════════
// EMERGENCY STOP ISR
// ═════════════════════════════════════════════
void IRAM_ATTR emergency_stop_ISR() {
    emergency_stop_active = true;
    digitalWrite(ESTOP_RELAY, LOW);   // Cut power!
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
}

// ═════════════════════════════════════════════
// FIRE JET
// ═════════════════════════════════════════════
void fire_jet(uint8_t jet_id, uint16_t duration_ms) {
    
    if (emergency_stop_active) return;
    
    int pin = -1;
    switch(jet_id) {
        case 1: pin = JET_1; break;
        case 2: pin = JET_2; break;
        case 3: pin = JET_3; break;
        case 4: pin = JET_4; break;
        case 5: pin = JET_5; break;
        case 6: pin = JET_6; break;
        default: return;
    }
    
    digitalWrite(pin, HIGH);
    delay(duration_ms);
    digitalWrite(pin, LOW);
}

// ═════════════════════════════════════════════
// READ SENSORS
// ═════════════════════════════════════════════
void read_and_send_sensors() {
    
    // Metal detection
    bool ind1 = !digitalRead(IND_SENSOR_1);
    bool ind2 = !digitalRead(IND_SENSOR_2);
    bool ind3 = !digitalRead(IND_SENSOR_3);
    metal_detected = ind1 || ind2 || ind3;
    
    // Metal "strength" (how many sensors triggered)
    float metal_strength = (ind1 + ind2 + ind3) / 3.0f;
    
    // Temperature
    float ambient = mlx.readAmbientTempC();
    float object_temp = mlx.readObjectTempC();
    bool thermal_alert = (object_temp - ambient) > 20.0;
    
    // Battery detection logic
    battery_suspected = (metal_detected && 
                         thermal_alert &&
                         metal_strength < 0.8);  // partial metal
    
    // Gas
    int gas_raw = analogRead(GAS_SENSOR);
    bool gas_alert = gas_raw > GAS_THRESHOLD;
    
    // Build JSON response
    StaticJsonDocument<256> doc;
    doc["metal_detected"]  = metal_detected;
    doc["metal_strength"]  = metal_strength;
    doc["is_ferrous"]      = ind1;  // ferrous sensor first
    doc["temperature"]     = object_temp;
    doc["thermal_alert"]   = thermal_alert;
    doc["battery_suspect"] = battery_suspected;
    doc["gas_level"]       = gas_raw;
    doc["gas_alert"]       = gas_alert;
    doc["emergency_stop"]  = emergency_stop_active;
    
    serializeJson(doc, Serial);
    Serial.println();
    
    // ⚠️ Immediate hazard response
    if (battery_suspected || gas_alert) {
        fire_jet(6, 200);   // HAZARD jet
        digitalWrite(LED_RED, HIGH);
        tone(BUZZER, 1000, 500);
    }
}

// ═════════════════════════════════════════════
// PROCESS COMMANDS FROM JETSON
// ═════════════════════════════════════════════
void process_command(const char* json_str) {
    
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, json_str);
    
    if (error) return;
    
    const char* cmd = doc["cmd"];
    
    if (strcmp(cmd, "eject") == 0) {
        uint8_t jet = doc["jet"];
        uint16_t duration = doc["duration_ms"] | 80;
        bool alarm = doc["alarm"] | false;
        
        if (alarm) {
            digitalWrite(LED_RED, HIGH);
            tone(BUZZER, 2000, 200);
        }
        
        fire_jet(jet, duration);
        
        if (alarm) {
            delay(500);
            digitalWrite(LED_RED, LOW);
            digitalWrite(LED_GREEN, HIGH);
        }
    }
    else if (strcmp(cmd, "emergency_eject") == 0) {
        // Immediate - no timing
        fire_jet(6, 300);
        tone(BUZZER, 3000, 1000);
        digitalWrite(LED_RED, HIGH);
    }
    else if (strcmp(cmd, "reset") == 0) {
        emergency_stop_active = false;
        digitalWrite(ESTOP_RELAY, HIGH);
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_GREEN, HIGH);
    }
    else if (strcmp(cmd, "test_jet") == 0) {
        uint8_t jet = doc["jet"] | 1;
        fire_jet(jet, 100);
    }
}

// ═════════════════════════════════════════════
void loop() {
    
    // Read sensors every 50ms
    static uint32_t last_sensor = 0;
    if (millis() - last_sensor > 50) {
        read_and_send_sensors();
        last_sensor = millis();
    }
    
    // Check for commands from Jetson
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        process_command(line.c_str());
    }
    
    // Process pending timed jets
    uint32_t now = millis();
    for (int i = 0; i < pending_count; i++) {
        if (now >= pending_jets[i].fire_time) {
            fire_jet(pending_jets[i].jet_id, 
                    pending_jets[i].duration_ms);
            // Remove from array
            pending_jets[i] = pending_jets[--pending_count];
            i--;
        }
    }
}
