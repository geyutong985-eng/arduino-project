// 主循环状态机
// 统一管所有状态，根据 Unity 指令 + 传感器判定切换

#include "ActuatorPneumatic.h"
#include "SensorIMU.h"
#include "SensorPPG.h"

// ===== 引脚定义 =====
#define PUMP_PIN 8
#define VALVE_PIN 9

// ===== 模块实例 =====
ActuatorPneumatic pneumatic;
SensorIMU imu;
SensorPPG ppg;

// ===== 主状态枚举 =====
enum MainState {
    STATE_IDLE,         // 空闲
    STATE_POSE_CHECK,   // 姿态判定
    STATE_ACTUATOR,   // 执行器控制
    STATE_FEEDBACK    // 反馈
};

MainState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

void setup() {
    Serial.begin(9600);
    pneumatic.init(PUMP_PIN, VALVE_PIN);

    Serial.println("Init IMU...");
    imu.init();
    delay(200);

    Serial.println("Init PPG...");
    ppg.init();
    delay(200);

    Serial.println("Request data...");
    imu.requestEuler();

    Serial.println("System ready");
}

// 临时测试用
// #define TEST_MODE
// #define TEST_IMU
#define TEST_PPG
// #define TEST_ALL

void loop() {
#ifdef TEST_PPG
    ppg.update();
    ppg.test();
    delay(20);  // 约 50Hz
#endif

#ifdef TEST_MODE
    pneumatic.test();
    while (true) { }
#endif

#ifndef TEST_MODE
#ifndef TEST_IMU
#ifndef TEST_PPG
    // 正式状态机
    switch (currentState) {
        case STATE_IDLE:
            currentState = STATE_POSE_CHECK;
            break;

        case STATE_POSE_CHECK:
            if (imu.getPitch() > 30) {
                currentState = STATE_ACTUATOR;
                stateStartTime = millis();
            }
            break;

        case STATE_ACTUATOR:
            if (millis() - stateStartTime < 3000) {
                pneumatic.inflate();
            } else {
                pneumatic.stop();
                currentState = STATE_FEEDBACK;
            }
            break;

        case STATE_FEEDBACK:
            Serial.println("DONE");
            currentState = STATE_IDLE;
            break;
    }
#endif
#endif
#endif
}