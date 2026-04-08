#ifndef SENSOR_FLEX_H
#define SENSOR_FLEX_H

#include <Arduino.h>

class SensorFlex {
public:
    // 构造函数：指定模拟输入引脚
    // pin: 连接的模拟引脚 (A0-A7)
    SensorFlex(uint8_t pin);

    // 初始化传感器（仅设置引脚模式，不进行校准）
    void begin();

    // 读取原始ADC值（10次平均滤波）
    int readRaw();

    // 读取原始ADC值（单次读取，无滤波）
    int readRawOnce();

    // 校准平展状态：用户摆好平展姿势后调用
    // 返回校准后的ADC均值
    int calibrateFlat();

    // 校准弯曲状态：用户摆好弯曲姿势后调用
    // 返回校准后的ADC均值
    int calibrateBent();

    // 双点校准：手动指定平展值和弯曲值
    void calibrate(int flatValue, int bentValue);

    // 检查是否已完成双点校准
    bool isCalibrated() const;

    // 获取归一化值 (0.0 - 1.0)
    // 未完成校准返回0.0
    float getNormalized();

    // 返回根据 flat/bent 双点校准线性映射得到的 0~180° 相对弯曲角度
    // 注意：这是映射角度，非物理量角器角度
    float readAngle();

    // 判断当前弯曲角度是否超过阈值
    // angleThreshold: 阈值角度 (0-180)
    bool isBentBeyond(float angleThreshold);

    // 检查校准差值是否足够大（用于判断传感器是否正确安装）
    bool isCalibrationValid(int minDifference = 80) const;

    // 获取校准差值（用于调试诊断）
    int getCalibrationDifference() const;

    // 打印校准状态诊断信息
    void printCalibrationInfo(Print& stream) const;

private:
    uint8_t _pin;           // 模拟输入引脚
    int _flatValue;         // 平展状态ADC值
    int _bentValue;         // 弯曲状态ADC值
    bool _flatCalibrated;   // 平展校准完成标志
    bool _bentCalibrated;   // 弯曲校准完成标志
};

#endif