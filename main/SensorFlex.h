#ifndef SENSOR_FLEX_H
#define SENSOR_FLEX_H

#include <Arduino.h>

// ========== 可配置参数 ==========
const int FLEX_FILTER_SAMPLES = 9;          // 滤波采样次数
const int FLEX_MIN_DIFF = 50;               // 最小校准差值
const float FLEX_ANGLE_THRESHOLD = 45.0;     // 弯曲阈值
const float FLEX_HYSTERESIS = 2.0;          // 迟滞范围
const unsigned long FLEX_PRINT_INTERVAL = 100; // 输出间隔(ms)

// ========== 状态枚举 ==========
enum FlexState {
    FLEX_FLAT,    // 伸直
    FLEX_MIDDLE,  // 中间
    FLEX_BENT     // 弯曲
};

class SensorFlex {
public:
    SensorFlex(int pin);

    void init();
    void calibrateFlat();
    void calibrateBent();
    void reset();
    void update();

    int getRaw() const;
    float getAngle() const;
    float getNormalized() const;
    FlexState getState() const;
    const char* getStateName() const;
    bool isCalibrated() const;
    bool isCalibrationValid() const;
    void printStatus() const;
    void test() const;

private:
    int flexPin;
    int flatValue;
    int bentValue;
    bool flatCalibrated;
    bool bentCalibrated;
    bool bentTriggered;
    float smoothRaw;
    unsigned long lastPrintTime;

    int readMedianRaw(int samples);
    int readStableRaw();
    int calibrateAverage(int samples);
    float getNormalized(int raw, int flatVal, int bentVal) const;
    float getAngle(int raw, int flatVal, int bentVal) const;
    void sortArray(int arr[], int n);
};

#endif
