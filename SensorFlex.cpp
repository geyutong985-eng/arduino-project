#include "SensorFlex.h"

SensorFlex::SensorFlex(uint8_t pin)
    : _pin(pin), _flatValue(0), _bentValue(0),
      _flatCalibrated(false), _bentCalibrated(false) {
}

void SensorFlex::begin() {
    pinMode(_pin, INPUT);
}

int SensorFlex::readRaw() {
    long sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += analogRead(_pin);
        delay(2);
    }
    return sum / 10;
}

int SensorFlex::readRawOnce() {
    return analogRead(_pin);
}

int SensorFlex::calibrateFlat() {
    _flatValue = readRaw();
    _flatCalibrated = true;
    return _flatValue;
}

int SensorFlex::calibrateBent() {
    _bentValue = readRaw();
    _bentCalibrated = true;
    return _bentValue;
}

void SensorFlex::calibrate(int flatValue, int bentValue) {
    _flatValue = flatValue;
    _bentValue = bentValue;
    _flatCalibrated = true;
    _bentCalibrated = true;
}

bool SensorFlex::isCalibrated() const {
    return _flatCalibrated && _bentCalibrated;
}

float SensorFlex::getNormalized() {
    if (!isCalibrated()) {
        return 0.0f;
    }
    int raw = readRaw();
    // 自动兼容两种方向：flat > bent 或 flat < bent
    float value;
    if (_flatValue > _bentValue) {
        // 平展值 > 弯曲值（正常情况）
        value = map(raw, _bentValue, _flatValue, 0, 1000) / 1000.0f;
    } else {
        // 平展值 < 弯曲值（方向相反）
        value = map(raw, _flatValue, _bentValue, 0, 1000) / 1000.0f;
    }
    return constrain(value, 0.0f, 1.0f);
}

float SensorFlex::readAngle() {
    if (!isCalibrated()) {
        return 0.0f;
    }
    float norm = getNormalized();
    // 确保在有效范围内
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    return norm * 180.0f;
}

bool SensorFlex::isBentBeyond(float angleThreshold) {
    return readAngle() >= angleThreshold;
}

// 检查校准差值是否足够大（用于判断传感器是否正确安装）
bool SensorFlex::isCalibrationValid(int minDifference) const {
    if (!isCalibrated()) return false;
    return abs(_flatValue - _bentValue) >= minDifference;
}

// 获取校准差值（用于调试诊断）
int SensorFlex::getCalibrationDifference() const {
    if (!isCalibrated()) return 0;
    return abs(_flatValue - _bentValue);
}

// 获取当前校准状态诊断信息
void SensorFlex::printCalibrationInfo(Print& stream) const {
    stream.print("Flex Calib - Flat: ");
    stream.print(_flatValue);
    stream.print(", Bent: ");
    stream.print(_bentValue);
    stream.print(", Diff: ");
    stream.print(getCalibrationDifference());
    stream.print(", Ready: ");
    stream.println(isCalibrated() ? "Yes" : "No");
}