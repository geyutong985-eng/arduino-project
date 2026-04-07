// 气动模块控制（气泵+气阀）
// 功能：控制充气和放气，给用户触觉反馈
// 引脚：D8-气泵（充气）, D9-气阀（放气）

#ifndef ACTUATOR_PNEUMATIC_H
#define ACTUATOR_PNEUMATIC_H

#include <Arduino.h>

class ActuatorPneumatic {
private:
    int pumpPin;   // 气泵引脚
    int valvePin;  // 气阀引脚

public:
    // 初始化，设置引脚
    void init(int pump, int valve) {
        pumpPin = pump;
        valvePin = valve;
        pinMode(pumpPin, OUTPUT);
        pinMode(valvePin, OUTPUT);
        // 初始状态：气泵关，气阀开（放气）
        stop();
    }

    // 充气：气泵开，气阀关
    void inflate() {
        digitalWrite(pumpPin, HIGH);
        digitalWrite(valvePin, LOW);
    }

    // 放气：气泵关，气阀开
    void deflate() {
        digitalWrite(pumpPin, LOW);
        digitalWrite(valvePin, HIGH);
    }

    // 停止：气泵关，气阀关
    void stop() {
        digitalWrite(pumpPin, LOW);
        digitalWrite(valvePin, LOW);
    }
};

#endif