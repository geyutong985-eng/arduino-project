// 震动模块控制
// 引脚: D6
// 触发条件: 弯曲传感器伸直 + 压力传感器未按下

#ifndef ACTUATOR_VIBRATOR_H
#define ACTUATOR_VIBRATOR_H

#include <Arduino.h>
#include "SensorFlex.h"

class ActuatorVibrator {
private:
    int motorPin;
    bool isVibrating;
    unsigned long vibrateStartTime;
    static const unsigned long VIBRATE_DURATION = 2000; // 震动持续2秒

public:
    ActuatorVibrator(int pin) : motorPin(pin), isVibrating(false), vibrateStartTime(0) {}

    void init() {
        pinMode(motorPin, OUTPUT);
        stop();
    }

    // 更新震动状态
    // 参数: flexState - 弯曲传感器状态, pressed - 压力传感器是否按下
    // 参数: flexReady - 弯曲传感器是否已校准
    void update(FlexState flexState, bool pressed, bool flexReady) {
        unsigned long now = millis();

        // 如果正在震动，检查是否超时
        if (isVibrating) {
            if (now - vibrateStartTime >= VIBRATE_DURATION) {
                stop();
                isVibrating = false;
            }
            return;  // 震动期间不检测新触发
        }

        // 只有弯曲传感器已校准后才检测触发
        if (!flexReady) return;

        // 触发条件：伸直 + 未按下
        bool shouldVibrate = (flexState == FLEX_FLAT) && (!pressed);

        if (shouldVibrate) {
            startVibrate();
        }
    }

    // 手动触发震动（测试用）
    void test() {
        Serial.println("[VIBRATOR] Test: 震动2秒...");
        startVibrate();
    }

    // 获取当前状态
    bool isActive() const {
        return isVibrating;
    }

private:
    void startVibrate() {
        digitalWrite(motorPin, HIGH);
        isVibrating = true;
        vibrateStartTime = millis();
    }

    void stop() {
        digitalWrite(motorPin, LOW);
    }
};

#endif