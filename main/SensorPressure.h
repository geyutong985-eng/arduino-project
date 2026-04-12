#ifndef SENSOR_PRESSURE_H
#define SENSOR_PRESSURE_H

#include <Arduino.h>

// ========== 可配置参数 ==========
const int PRESSURE_FILTER_WINDOW = 9;           // 中值滤波窗口大小(奇数)
const unsigned long PRESSURE_PRINT_INTERVAL = 500; // 输出间隔(ms)

class SensorPressure {
public:
    SensorPressure(int pin);

    void init();
    void update();

    int getRaw() const;              // 获取滤波后的值
    int getUnfilteredRaw() const;    // 获取原始值
    bool isPressed() const;           // 是否按下（超过阈值）
    void setThreshold(int threshold); // 设置按下阈值
    void test() const;               // 测试输出

private:
    int pressurePin;
    int medianBuffer[PRESSURE_FILTER_WINDOW];
    int bufferIndex;
    bool bufferFilled;
    int filteredValue;
    int pressThreshold;
    unsigned long lastPrintTime;

    int readMedian();
};

#endif