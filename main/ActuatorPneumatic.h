// 手掌气动模块控制（气泵+气阀）
// 功能：控制充气/放气，由压力传感器触发
// 引脚：D8-气泵（充气）, D9-气阀（放气）/ 压力按下 → 充气 / 松开 → 放气

#ifndef ACTUATOR_PNEUMATIC_H
#define ACTUATOR_PNEUMATIC_H

#include <Arduino.h>
#include "PneumaticState.h"

class ActuatorPneumatic {
private:
    int pumpPin;   // 气泵引脚
    int valvePin; // 气阀引脚
    PneumaticState state;

    // 阈值参数
    int targetPressure;  // 目标压力（充气触发值）
    int minPressure;  // 最小压力（放气完成值）
    unsigned long holdDuration; // 维持时长(ms)

    unsigned long stateStartTime; // 状态开始时间

public:
    // 初始化
    void init(int pump, int valve) {
        pumpPin = pump;
        valvePin = valve;
        pinMode(pumpPin, OUTPUT);
        pinMode(valvePin, OUTPUT);

        // 默认参数
        targetPressure = 100;  // TODO: 根据传感器调整
        minPressure = 10;
        holdDuration = 3000;

        stop();
        state = PNEUMATIC_IDLE;
        stateStartTime = millis();
    }

    // 设置阈值参数
    void setConfig(int target, int min, unsigned long holdMs) {
        targetPressure = target;
        minPressure = min;
        holdDuration = holdMs;
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

    // 更新状态（传入当前传感器值）
    // 由主循环每次调用
    void update(int sensorValue) {
        switch (state) {
            case PNEUMATIC_IDLE:
                // 空闲：传感器值达到目标则开始充气
                if (sensorValue >= targetPressure) {
                    startInflate();
                }
                break;

            case PNEUMATIC_INFLATING:
                inflate();
                // 充气完成（或已达到目标）则切换维持
                if (sensorValue >= targetPressure) {
                    state = PNEUMATIC_HOLDING;
                    stateStartTime = millis();
                }
                break;

            case PNEUMATIC_HOLDING:
                stop();  // 保持气压
                // 维持时间到则切换放气
                if (millis() - stateStartTime >= holdDuration) {
                    startDeflate();
                }
                break;

            case PNEUMATIC_DEFLATING:
                deflate();
                // 放气完成则切换空闲
                if (sensorValue <= minPressure) {
                    state = PNEUMATIC_IDLE;
                    stop();
                }
                break;
        }
    }

    // 获取当前状态
    PneumaticState getState() {
        return state;
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

    // 测试：充气3s -> 维持3s -> 放气3s
    // 可单独运行测试气动功能
    void test() {
        Serial.println("Test: Inflating...");
        inflate();
        delay(3000);

        Serial.println("Test: Holding...");
        stop();
        delay(3000);

        Serial.println("Test: Deflating...");
        deflate();
        delay(3000);

        stop();
        Serial.println("Test done");
    }
};

#endif