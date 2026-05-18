// 小臂气动模块控制（气泵+气阀）
// 功能：控制充气/放气，由IMU姿态触发
// 引脚：D11-气泵（充气）, D12-气阀（放气）
// 触发条件：IMU半抬 → 充气9秒 → 压力按下后松开放气

#ifndef ACTUATOR_PNEUMATIC2_H
#define ACTUATOR_PNEUMATIC2_H

#include <Arduino.h>
#include "PneumaticState.h"

class ActuatorPneumatic2 {
private:
    int pumpPin;   // 气泵引脚
    int valvePin; // 气阀引脚
    PneumaticState state;

    unsigned long stateStartTime; // 状态开始时间
    unsigned long inflateDuration;   // 充气持续时间(ms)，默认9秒

public:
    // 初始化
    void init(int pump, int valve) {
        pumpPin = pump;
        valvePin = valve;
        pinMode(pumpPin, OUTPUT);
        pinMode(valvePin, OUTPUT);

        // 默认4秒充气
        inflateDuration = 4000;

        stop();
        state = PNEUMATIC_IDLE;
        stateStartTime = millis();
    }

    // 设置充气时间
    void setInflateDuration(unsigned long ms) {
        inflateDuration = ms;
    }

    // 手动触发充气
    void startInflate() {
        state = PNEUMATIC_INFLATING;
        stateStartTime = millis();
        inflate();
    }

    // 手动触发放气
    void startDeflate() {
        state = PNEUMATIC_DEFLATING;
        stateStartTime = millis();
        deflate();
    }

    // 更新状态（由主循环调用，处理充气计时）
    void update() {
        if (state == PNEUMATIC_INFLATING) {
            // 充气达到设定时间后切换维持，停止气泵
            if (millis() - stateStartTime >= inflateDuration) {
                state = PNEUMATIC_HOLDING;
                stateStartTime = millis();
                stop();  // 停止充气，保持气压
            }
        }
    }

    // 检查是否需要从充气/维持切换到放气（压力按下后松手）
    void checkStopOnPressureRelease(bool wasPressed, bool isReleased) {
        if (wasPressed && isReleased &&
            (state == PNEUMATIC_INFLATING || state == PNEUMATIC_HOLDING)) {
            startDeflate();
        }
    }

    // 获取当前状态
    PneumaticState getState() {
        return state;
    }

    // 判断是否正在充气或维持
    bool isActive() {
        return state == PNEUMATIC_INFLATING || state == PNEUMATIC_HOLDING;
    }

    // 底层控制方法
    void inflate() {
        digitalWrite(pumpPin, HIGH);
        digitalWrite(valvePin, LOW);
    }

    void deflate() {
        digitalWrite(pumpPin, LOW);
        digitalWrite(valvePin, HIGH);
    }

    void stop() {
        digitalWrite(pumpPin, LOW);
        digitalWrite(valvePin, LOW);
    }

    // 测试：充气3秒 -> 放气
    void test() {
        Serial.println("[Pneumatic2] Test: Inflating...");
        startInflate();
        delay(3000);
        Serial.println("[Pneumatic2] Test: Deflating...");
        startDeflate();
        delay(1000);
        stop();
        Serial.println("[Pneumatic2] Test done");
    }
};

#endif