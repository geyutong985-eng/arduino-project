#include "SensorFlex.h"

SensorFlex::SensorFlex(int pin) : flexPin(pin), flatValue(0), bentValue(0),
    flatCalibrated(false), bentCalibrated(false), bentTriggered(false),
    smoothRaw(-1), lastPrintTime(0) {}

void SensorFlex::init() {
    pinMode(flexPin, INPUT);
    Serial.println("=== Elbow Flex Sensor Sensitive Test ===");
    Serial.println("f -> 校准伸直  b -> 校准弯曲  s -> 状态  r -> 重置");
}

void SensorFlex::calibrateFlat() {
    flatValue = calibrateAverage(FLEX_FILTER_SAMPLES * 3);
    flatCalibrated = true;
    smoothRaw = -1;
    bentTriggered = false;
    Serial.println(">> 手臂伸直校准完成");
    Serial.print("Flat = ");
    Serial.println(flatValue);
}

void SensorFlex::calibrateBent() {
    bentValue = calibrateAverage(FLEX_FILTER_SAMPLES * 3);
    bentCalibrated = true;
    smoothRaw = -1;
    bentTriggered = false;
    Serial.println(">> 手肘弯曲校准完成");
    Serial.print("Flat = ");
    Serial.print(flatValue);
    Serial.print(" | Bent = ");
    Serial.print(bentValue);
    Serial.print(" | Diff = ");
    Serial.print(abs(flatValue - bentValue));
    Serial.print(" | Valid: ");
    Serial.println(isCalibrationValid() ? "Yes" : "No");
}

void SensorFlex::reset() {
    flatValue = bentValue = 0;
    flatCalibrated = bentCalibrated = false;
    smoothRaw = -1;
    bentTriggered = false;
    Serial.println(">> 已重置校准");
}

void SensorFlex::update() {
    if (!isCalibrated()) return;
    if (!isCalibrationValid()) return;

    unsigned long now = millis();
    if (now - lastPrintTime < FLEX_PRINT_INTERVAL) return;
    lastPrintTime = now;

    int raw = readStableRaw();
    float norm = getNormalized(raw, flatValue, bentValue);
    float angle = getAngle(raw, flatValue, bentValue);

    if (!bentTriggered && angle >= (FLEX_ANGLE_THRESHOLD + FLEX_HYSTERESIS)) {
        bentTriggered = true;
    } else if (bentTriggered && angle <= (FLEX_ANGLE_THRESHOLD - FLEX_HYSTERESIS)) {
        bentTriggered = false;
    }

    Serial.print("Raw: ");
    Serial.print(raw);
    Serial.print("  Angle: ");
    Serial.print(angle, 1);
    Serial.print("  Norm: ");
    Serial.print(norm, 3);
    Serial.print("  State: ");
    Serial.print(getStateName());
    if (bentTriggered) Serial.print("  >> 弯曲!");
    Serial.println();
}

int SensorFlex::getRaw() const {
    return const_cast<SensorFlex*>(this)->readStableRaw();
}

float SensorFlex::getAngle() const {
    int raw = const_cast<SensorFlex*>(this)->readStableRaw();
    return getAngle(raw, flatValue, bentValue);
}

float SensorFlex::getNormalized() const {
    int raw = const_cast<SensorFlex*>(this)->readStableRaw();
    return getNormalized(raw, flatValue, bentValue);
}

FlexState SensorFlex::getState() const {
    if (!isCalibrated()) return FLEX_FLAT;
    if (bentTriggered) return FLEX_BENT;
    int raw = const_cast<SensorFlex*>(this)->readStableRaw();
    float angle = getAngle(raw, flatValue, bentValue);
    if (angle <= 5.0) return FLEX_FLAT;
    return FLEX_MIDDLE;
}

bool SensorFlex::isCalibrated() const {
    return flatCalibrated && bentCalibrated;
}

bool SensorFlex::isCalibrationValid() const {
    if (!flatCalibrated || !bentCalibrated) return false;
    return abs(flatValue - bentValue) >= FLEX_MIN_DIFF;
}

void SensorFlex::printStatus() const {
    Serial.println("===== 校准状态 =====");
    if (!flatCalibrated) {
        Serial.println("Flat: 未校准");
    } else {
        Serial.print("Flat = ");
        Serial.println(flatValue);
    }
    if (!bentCalibrated) {
        Serial.println("Bent: 未校准");
    } else {
        Serial.print("Bent = ");
        Serial.println(bentValue);
    }
    if (isCalibrated()) {
        Serial.print("Diff = ");
        Serial.println(abs(flatValue - bentValue));
        Serial.print("Valid: ");
        Serial.println(isCalibrationValid() ? "Yes" : "No");
    }
}

void SensorFlex::test() const {
    Serial.println("===== Flex Sensor Test =====");
    printStatus();
    if (isCalibrated() && isCalibrationValid()) {
        int raw = const_cast<SensorFlex*>(this)->readStableRaw();
        float angle = getAngle(raw, flatValue, bentValue);
        FlexState state = getState();
        Serial.print("Current -> Raw: ");
        Serial.print(raw);
        Serial.print(" | Angle: ");
        Serial.print(angle, 1);
        Serial.print(" | State: ");
        Serial.println(state == FLEX_FLAT ? "伸直" : (state == FLEX_MIDDLE ? "中间" : "弯曲"));
    }
}

const char* SensorFlex::getStateName() const {
    if (bentTriggered) return "弯曲";
    int raw = const_cast<SensorFlex*>(this)->readStableRaw();
    float angle = getAngle(raw, flatValue, bentValue);
    if (angle <= 5.0) return "伸直";
    return "中间";
}

// ========== 私有方法 ==========

void SensorFlex::sortArray(int arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - 1 - i; j++) {
            if (arr[j] > arr[j + 1]) {
                int t = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = t;
            }
        }
    }
}

int SensorFlex::readMedianRaw(int samples) {
    if (samples > 25) samples = 25;
    int values[25];
    for (int i = 0; i < samples; i++) {
        values[i] = analogRead(flexPin);
        delay(2);
    }
    sortArray(values, samples);
    return values[samples / 2];
}

int SensorFlex::readStableRaw() {
    int medianRaw = readMedianRaw(FLEX_FILTER_SAMPLES);
    if (smoothRaw < 0) {
        smoothRaw = medianRaw;
    } else {
        smoothRaw = 0.75 * smoothRaw + 0.25 * medianRaw;
    }
    return (int)(smoothRaw + 0.5);
}

int SensorFlex::calibrateAverage(int samples) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(flexPin);
        delay(4);
    }
    return sum / samples;
}

float SensorFlex::getNormalized(int raw, int flatVal, int bentVal) const {
    if (flatVal == bentVal) return 0.0;
    float norm = (float)(raw - flatVal) / (float)(bentVal - flatVal);
    return constrain(norm, 0.0, 1.0);
}

float SensorFlex::getAngle(int raw, int flatVal, int bentVal) const {
    return getNormalized(raw, flatVal, bentVal) * 180.0;
}