#include "SensorPressure.h"

SensorPressure::SensorPressure(int pin) : pressurePin(pin), bufferIndex(0),
    bufferFilled(false), filteredValue(0), pressThreshold(600), lastPrintTime(0) {}

void SensorPressure::init() {
    pinMode(pressurePin, INPUT);

    // 检查引脚模式
    Serial.print("=== Pressure Sensor (Pin ");
    Serial.print(pressurePin);
    Serial.println(") ===");

    // 初始化缓冲区：用第一次读数填满
    int firstVal = analogRead(pressurePin);
    Serial.print("First Read: ");
    Serial.println(firstVal);
    for (int i = 0; i < PRESSURE_FILTER_WINDOW; i++) {
        medianBuffer[i] = firstVal;
    }
    bufferIndex = 0;
    bufferFilled = true;

    filteredValue = firstVal;
}

void SensorPressure::update() {
    unsigned long now = millis();
    if (now - lastPrintTime < PRESSURE_PRINT_INTERVAL) return;
    lastPrintTime = now;


    // 读取并滤波
    filteredValue = readMedian();

    // 输出
    Serial.print("[PRESSURE] Raw: ");
    Serial.print(filteredValue);
    Serial.print("  Pressed: ");
    Serial.println(isPressed() ? "YES" : "NO");
}

int SensorPressure::getRaw() const {
    return filteredValue;
}

int SensorPressure::getUnfilteredRaw() const {
    return analogRead(pressurePin);
}

bool SensorPressure::isPressed() const {
    return filteredValue < pressThreshold;
}

void SensorPressure::setThreshold(int threshold) {
    pressThreshold = threshold;
}

void SensorPressure::test() const {
    Serial.println("===== Pressure Sensor Test =====");
    Serial.print("Filtered: ");
    Serial.println(filteredValue);
    Serial.print("Threshold: ");
    Serial.println(pressThreshold);
    Serial.print("Pressed: ");
    Serial.println(isPressed() ? "YES" : "NO");
}

// ========== 私有方法 ==========

int SensorPressure::readMedian() {
    // 1. 读取新原始值
    int newRaw = analogRead(pressurePin);

    // 2. 更新环形缓冲区
    medianBuffer[bufferIndex] = newRaw;
    bufferIndex = (bufferIndex + 1) % PRESSURE_FILTER_WINDOW;
    if (!bufferFilled && bufferIndex == 0) {
        bufferFilled = true;
    }

    // 3. 复制并排序
    int temp[PRESSURE_FILTER_WINDOW];
    memcpy(temp, medianBuffer, sizeof(medianBuffer));

    for (int i = 0; i < PRESSURE_FILTER_WINDOW - 1; i++) {
        for (int j = i + 1; j < PRESSURE_FILTER_WINDOW; j++) {
            if (temp[i] > temp[j]) {
                int t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    return temp[PRESSURE_FILTER_WINDOW / 2];
}