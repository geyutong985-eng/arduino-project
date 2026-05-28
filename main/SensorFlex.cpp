#include "SensorFlex.h"

SensorFlex::SensorFlex(int pin) : flexPin(pin), flatValue(0), bentValue(0),
    flatCalibrated(false), bentCalibrated(false), bentTriggered(false),
    detailedState(FLEX_DETAILED_FLAT), candidateState(FLEX_DETAILED_FLAT),
    candidateCount(0), currentRaw(0), currentAngle(0.0),
    smoothRaw(-1), lastSampleTime(0), lastPrintTime(0) {}

void SensorFlex::init() {
    pinMode(flexPin, INPUT);
    Serial.println("=== Elbow Flex Sensor ===");
}

void SensorFlex::calibrateFlat() {
    flatValue = calibrateAverage(25);
    flatCalibrated = true;
    smoothRaw = -1;
    bentTriggered = false;
    detailedState = FLEX_DETAILED_FLAT;
    candidateState = FLEX_DETAILED_FLAT;
    candidateCount = 0;
    Serial.println(">> 伸直校准完成");
    Serial.print("Flat = ");
    Serial.println(flatValue);
}

void SensorFlex::calibrateBent() {
    bentValue = calibrateAverage(25);
    bentCalibrated = true;
    smoothRaw = -1;
    bentTriggered = false;
    detailedState = FLEX_DETAILED_FLAT;
    candidateState = FLEX_DETAILED_FLAT;
    candidateCount = 0;
    Serial.println(">> 弯曲校准完成");
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
    detailedState = FLEX_DETAILED_FLAT;
    candidateState = FLEX_DETAILED_FLAT;
    candidateCount = 0;
    Serial.println(">> 已重置校准");
}

void SensorFlex::update() {
    if (!isCalibrated()) return;
    if (!isCalibrationValid()) return;

    unsigned long now = millis();
    if (now - lastSampleTime >= FLEX_SAMPLE_INTERVAL) {
        lastSampleTime = now;
        currentRaw = readStableRaw();
        currentAngle = getAngle(currentRaw, flatValue, bentValue);

        FlexDetailedState measuredState = detailedState;
        if (detailedState == FLEX_DETAILED_BENT) {
            measuredState = currentAngle <= FLEX_BENT_OFF_ANGLE ? FLEX_DETAILED_FLAT : FLEX_DETAILED_BENT;
        } else {
            measuredState = currentAngle >= FLEX_BENT_ON_ANGLE ? FLEX_DETAILED_BENT : FLEX_DETAILED_FLAT;
        }

        if (measuredState == detailedState) {
            candidateState = detailedState;
            candidateCount = 0;
        } else if (measuredState != candidateState) {
            candidateState = measuredState;
            candidateCount = 1;
        } else {
            candidateCount++;
            if (candidateCount >= FLEX_STABLE_SAMPLES) {
                detailedState = candidateState;
                bentTriggered = detailedState == FLEX_DETAILED_BENT;
                candidateCount = 0;
            }
        }
    }

    if (now - lastPrintTime < FLEX_PRINT_INTERVAL) {
        return;
    }
    lastPrintTime = now;

    Serial.print("[FLEX] Raw: ");
    Serial.print(currentRaw);
    Serial.print("  Angle: ");
    Serial.print(currentAngle, 1);
    Serial.print("  State: ");
    Serial.print(getStateName());
    if (bentTriggered) Serial.print(" >>BENT");
    Serial.println();
}

int SensorFlex::getRaw() const {
    if (isCalibrated() && isCalibrationValid()) {
        return currentRaw;
    }
    return const_cast<SensorFlex*>(this)->readStableRaw();
}

float SensorFlex::getAngle() const {
    if (isCalibrated() && isCalibrationValid()) {
        return currentAngle;
    }
    int raw = const_cast<SensorFlex*>(this)->readStableRaw();
    return getAngle(raw, flatValue, bentValue);
}

float SensorFlex::getNormalized() const {
    if (isCalibrated() && isCalibrationValid()) {
        return getNormalized(currentRaw, flatValue, bentValue);
    }
    int raw = const_cast<SensorFlex*>(this)->readStableRaw();
    return getNormalized(raw, flatValue, bentValue);
}

FlexState SensorFlex::getState() const {
    if (!isCalibrated()) return FLEX_MIDDLE;

    return detailedState == FLEX_DETAILED_BENT ? FLEX_BENT : FLEX_FLAT;
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
    return detailedState == FLEX_DETAILED_BENT ? "弯曲" : "伸直";
}

FlexDetailedState SensorFlex::getDetailedState() {
    if (!isCalibrated() || !isCalibrationValid()) return FLEX_DETAILED_FLAT;
    return detailedState;
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
