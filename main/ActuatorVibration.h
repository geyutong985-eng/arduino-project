// 震动马达控制
// 功能：控制震动马达，给用户触觉反馈
// 引脚：D6

#ifndef ACTUATOR_VIBRATION_H
#define ACTUATOR_VIBRATION_H

#include <Arduino.h>

enum VibrationState {
    VIBRATION_IDLE,  // 空闲
    VIBRATION_ON     // 震动中
};

class ActuatorVibration {
private:
    int motorPin;
    VibrationState state;
    unsigned long startTime;
    unsigned long duration;  // 震动持续时长(ms)
    int strength;  // 震动强度 0-255

public:
    void init(int pin) {
        motorPin = pin;
        pinMode(motorPin, OUTPUT);
        duration = 2000;  // 默认2秒
        strength = 80;   // 默认强度 (0-255)
        stop();
    }

    void setDuration(unsigned long ms) {
        duration = ms;
    }

    void setStrength(int s) {
        strength = constrain(s, 0, 255);
    }

    void start() {
        if (state == VIBRATION_ON) return;  // 已在震动中
        state = VIBRATION_ON;
        startTime = millis();
        analogWrite(motorPin, strength);  // PWM 控制强度
    }

    void stop() {
        state = VIBRATION_IDLE;
        digitalWrite(motorPin, LOW);
    }

    void update() {
        if (state == VIBRATION_ON && millis() - startTime >= duration) {
            stop();
        }
    }

    bool isActive() {
        return state == VIBRATION_ON;
    }
};

#endif
