// ═══════════════════════════════════════════════
// ESP32 #1 - SENSOR NODE
// sensors.ino
// ═══════════════════════════════════════════════

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "SparkFun_AS7265X.h"
#include "HX711.h"

// ── SENSORS ───────────────────────────────────
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, 
                       TCS34725_GAIN_4X);
AS7265X spectral;
HX711 scale;

// ── PINS ──────────────────────────────────────
#define IR_1        34  // Entry
#define IR_2        35  // Pre-camera
#define IR_3        36  // At-camera  
#define IR_4        39  // Post-camera
#define ULTRASONIC_TRIG  13
#define ULTRASONIC_ECHO  12
#define HX711_DOUT   16
#define HX711_SCK    17

// ── STATE ─────────────────────────────────────
bool item_present = false;
uint32_t item_entry_time = 0;

// ═════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);  // SDA=21, SCL=22
    
    // Presence sensors
    pinMode(IR_1, INPUT);
    pinMode(IR_2, INPUT);
    pinMode(IR_3, INPUT);
    pinMode(IR_4, INPUT);
    
    // Ultrasonic
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    
    // Color sensor
    if (tcs.begin()) {
        Serial.println("{\"status\":\"TCS34725_ok\"}");
    }
    
    // Spectral sensor AS7265x
    if (spectral.begin() == false) {
        Serial.println("{\"status\":\"AS7265x_error\"}");
    } else {
        spectral.setMeasurementMode(
            AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT
        );
        spectral.setGain(AS7265X_GAIN_64X);
        Serial.println("{\"status\":\"AS7265x_ok\"}");
    }
    
    // Weight
    scale.begin(HX711_DOUT, HX711_SCK);
    scale.set_scale(2280.f);  // calibrate!
    scale.tare();
    
    Serial.println("{\"status\":\"sensors_ready\"}");
}

// ═════════════════════════════════════════════
float measure_distance() {
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    
    long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
    return duration * 0.034 / 2;  // cm
}

// ═════════════════════════════════════════════
void read_all_sensors() {
    
    // Presence
    bool ir1 = !digitalRead(IR_1);
    bool ir2 = !digitalRead(IR_2);
    bool ir3 = !digitalRead(IR_3);
    bool ir4 = !digitalRead(IR_4);
    
    // Track item entry
    if (ir1 && !item_present) {
        item_present = true;
        item_entry_time = millis();
    }
    if (!ir1 && item_present) {
        item_present = false;
    }
    
    // Distance / Height
    float distance = measure_distance();
    float item_height = 20.0 - distance;  // 20cm = belt surface
    item_height = max(0.0f, item_height);
    
    // Color
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    
    // Spectral (AS7265x)
    spectral.takeMeasurements();
    
    // Weight
    float weight = 0;
    if (scale.is_ready()) {
        weight = scale.get_units(3);  // average of 3
    }
    
    // Build JSON
    StaticJsonDocument<512> doc;
    
    // Presence
    doc["ir_entry"]     = ir1;
    doc["ir_precam"]    = ir2;
    doc["ir_camera"]    = ir3;
    doc["ir_postjet"]   = ir4;
    doc["item_present"] = ir1 || ir2 || ir3;
    
    // Physical
    doc["height_cm"]    = item_height;
    doc["weight_g"]     = weight;
    doc["distance_cm"]  = distance;
    
    // Color (normalized)
    float total = c + 1;
    doc["color_r"] = r / total;
    doc["color_g"] = g / total;
    doc["color_b"] = b / total;
    doc["color_c"] = c;
    doc["is_dark"]  = (c < 100);  // dark/black object
    
    // Spectral (18 channels)
    JsonObject spectrum = doc.createNestedObject("spectrum");
    spectrum["410nm"] = spectral.getCalibratedA();
    spectrum["435nm"] = spectral.getCalibratedB();
    spectrum["460nm"] = spectral.getCalibratedC();
    spectrum["485nm"] = spectral.getCalibratedD();
    spectrum["510nm"] = spectral.getCalibratedE();
    spectrum["535nm"] = spectral.getCalibratedF();
    // AS7265x chip 2
    spectrum["560nm"] = spectral.getCalibratedG();
    spectrum["585nm"] = spectral.getCalibratedH();
    spectrum["610nm"] = spectral.getCalibratedR();
    spectrum["645nm"] = spectral.getCalibratedI();
    spectrum["680nm"] = spectral.getCalibratedS();
    spectrum["705nm"] = spectral.getCalibratedJ();
    // AS7265x chip 3 (NIR)
    spectrum["730nm"] = spectral.getCalibratedT();
    spectrum["760nm"] = spectral.getCalibratedU();
    spectrum["810nm"] = spectral.getCalibratedV();
    spectrum["860nm"] = spectral.getCalibratedW();
    spectrum["900nm"] = spectral.getCalibratedX(); // if available
    spectrum["940nm"] = spectral.getCalibratedY();
    
    serializeJson(doc, Serial);
    Serial.println();
}

// ═════════════════════════════════════════════
void loop() {
    
    static uint32_t last_read = 0;
    
    // Read sensors at 20Hz
    if (millis() - last_read > 50) {
        read_all_sensors();
        last_read = millis();
    }
    
    // Check commands from Jetson
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        if (cmd == "tare") {
            scale.tare();
            Serial.println("{\"status\":\"tared\"}");
        }
        else if (cmd == "calibrate") {
            // Put known weight and calibrate
            Serial.println("{\"status\":\"calibrating\"}");
        }
    }
}
