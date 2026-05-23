// 震动马达控制
// 功能：控制震动马达，给用户触觉反馈
// 支持普通震动和脉冲模式（1秒开-2秒停循环）
// 引脚：ESP32 GPIO

#ifndef ACTUATOR_VIBRATION_H
#define ACTUATOR_VIBRATION_H

#include <Arduino.h>

// 检测是否为 ESP32 平台
#if defined(ESP32)
#define IS_ESP32 1
#else
#define IS_ESP32 0
#endif

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

    // ESP32 LEDC 通道
#if IS_ESP32
    int ledcChannel;
    static int nextLedcChannel;
#endif

public:
    void init(int pin) {
        motorPin = pin;
#if IS_ESP32
        // 分配 LEDC 通道
        ledcChannel = nextLedcChannel++;
        // 配置 LEDC 通道: 1kHz, 8-bit resolution
        ledcSetup(ledcChannel, 1000, 8);
        // 绑定引脚
        ledcAttachPin(motorPin, ledcChannel);
        ledcWrite(ledcChannel, 0);
#else
        pinMode(motorPin, OUTPUT);
        digitalWrite(motorPin, LOW);
#endif
        duration = 2000;  // 默认2秒
        strength = 80;    // 默认强度 (0-255)

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

    // 写入 PWM（ESP32 用 LEDC，UNO 用 analogWrite）
    void writePwm(int value) {
#if IS_ESP32
        ledcWrite(ledcChannel, value);
#else
        analogWrite(motorPin, value);
#endif
    }

    // 普通震动模式
    void start() {
        if (state == VIBRATION_ON || state == VIBRATION_PULSE_ON) return;
        pulseModeActive = false;
        state = VIBRATION_ON;
        startTime = millis();
        writePwm(strength);
    }

    void stop() {
        state = VIBRATION_IDLE;
        pulseModeActive = false;
#if IS_ESP32
        ledcWrite(ledcChannel, 0);
#else
        digitalWrite(motorPin, LOW);
#endif
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
        writePwm(strength);
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
#if IS_ESP32
                ledcWrite(ledcChannel, 0);
#else
                digitalWrite(motorPin, LOW);
#endif
            }
        } else {
            // 当前是关，检查是否需要打开（2秒后）
            if (now - lastPulseToggle >= 2000) {
                pulseStateOn = true;
                lastPulseToggle = now;
                state = VIBRATION_PULSE_ON;
                writePwm(strength);
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

// ESP32 LEDC 通道计数器
#if IS_ESP32
int ActuatorVibration::nextLedcChannel = 0;
#endif

#endif
