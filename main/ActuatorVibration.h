// 震动马达控制
// 功能：控制震动马达，给用户触觉反馈
// 支持普通震动和脉冲模式（1秒开-2秒停循环）
// 引脚：D6 或 D10

#ifndef ACTUATOR_VIBRATION_H
#define ACTUATOR_VIBRATION_H

#include <Arduino.h>

enum VibrationState {
    VIBRATION_IDLE,     // 空闲
    VIBRATION_ON,       // 震动中（普通模式）
    VIBRATION_PULSE_ON, // 脉冲：开
    VIBRATION_PULSE_OFF // 脉冲：关
};

class ActuatorVibration {
private:
    int motorPin;
    VibrationState state;
    unsigned long startTime;
    unsigned long duration;  // 震动持续时长(ms)
    int strength;           // 震动强度 0-255

    // 脉冲模式变量
    bool pulseModeActive;
    unsigned long pulseCycleStart;
    unsigned long lastPulseToggle;
    bool pulseStateOn;

public:
    void init(int pin) {
        motorPin = pin;
        pinMode(motorPin, OUTPUT);
        digitalWrite(motorPin, LOW);  // 先设置为 LOW
        duration = 2000;  // 默认2秒
        strength = 80;    // 默认强度 (0-255)

        // 确保初始化为关闭状态
        state = VIBRATION_IDLE;
        pulseModeActive = false;
        pulseStateOn = false;
    }

    void setDuration(unsigned long ms) {
        duration = ms;
    }

    void setStrength(int s) {
        strength = constrain(s, 0, 255);
    }

    // 普通震动模式
    void start() {
        if (state == VIBRATION_ON || state == VIBRATION_PULSE_ON) return;
        pulseModeActive = false;
        state = VIBRATION_ON;
        startTime = millis();
        analogWrite(motorPin, strength);
    }

    void stop() {
        state = VIBRATION_IDLE;
        pulseModeActive = false;
        digitalWrite(motorPin, LOW);
    }

    void update() {
        // 普通模式计时
        if (state == VIBRATION_ON && millis() - startTime >= duration) {
            stop();
        }
        // 脉冲模式处理
        if (pulseModeActive) {
            updatePulse();
        }
    }

    // 启动脉冲模式
    void startPulse() {
        if (pulseModeActive) return;
        pulseModeActive = true;
        pulseStateOn = true;
        pulseCycleStart = millis();
        lastPulseToggle = millis();
        state = VIBRATION_PULSE_ON;
        analogWrite(motorPin, strength);
    }

    // 更新脉冲模式
    void updatePulse() {
        unsigned long now = millis();

        if (pulseStateOn) {
            // 当前是开，检查是否需要关闭（1秒后）
            if (now - lastPulseToggle >= 1000) {
                pulseStateOn = false;
                lastPulseToggle = now;
                state = VIBRATION_PULSE_OFF;
                digitalWrite(motorPin, LOW);
            }
        } else {
            // 当前是关，检查是否需要打开（2秒后）
            if (now - lastPulseToggle >= 2000) {
                pulseStateOn = true;
                lastPulseToggle = now;
                state = VIBRATION_PULSE_ON;
                analogWrite(motorPin, strength);
            }
        }
    }

    // 停止脉冲模式
    void stopPulse() {
        pulseModeActive = false;
        stop();
    }

    bool isActive() {
        return state == VIBRATION_ON || state == VIBRATION_PULSE_ON;
    }

    bool isPulseMode() {
        return pulseModeActive;
    }
};

#endif
