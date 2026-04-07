// Main loop
// Test: Pneumatic module inflate/deflate

#include "ActuatorPneumatic.h"

ActuatorPneumatic pneumatic;

// Pump on D8, Valve on D9
#define PUMP_PIN 8
#define VALVE_PIN 9

void setup() {
    Serial.begin(9600);
    pneumatic.init(PUMP_PIN, VALVE_PIN);
    Serial.println("Pneumatic test start");
}

void loop() {
    // Inflate for 3 seconds
    Serial.println("Inflating...");
    pneumatic.inflate();
    delay(3000);

    // Deflate for 3 seconds
    Serial.println("Deflating...");
    pneumatic.deflate();
    delay(3000);
}