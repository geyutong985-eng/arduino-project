// 主循环状态机
// 统一管所有状态，根据 Unity 指令 + 传感器判定切换

#include "ActuatorPneumatic.h"
#include "SensorFlex.h"
#include "SensorIMU.h"
#include "SensorEMG.h"
#include "UnityComm.h"

// ===== 引脚定义 =====
#define PUMP_PIN 8
#define VALVE_PIN 9
#define VIBRATE_PIN 10

// ===== 模块实例 =====
ActuatorPneumatic pneumatic;
SensorFlex flex;
SensorIMU imu;
SensorEMG emg;
UnityComm unity;

// ===== 主状态枚举 =====
enum MainState {
    STATE_IDLE,         // 空闲，等待 Unity 指令
    STATE_POSE_CHECK,   // 姿态判定
    STATE_BIO_MONITOR,  // 生理数据监测
    STATE_ACTUATOR,   // 执行器控制（气动+震动）
    STATE_FEEDBACK    // 反馈给 Unity
};

MainState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

// ===== 阈值参数（可由 Unity 动态设置）=====
int poseThreshold = 30;      // 姿态角度阈值
int bioThreshold = 50;        // 生理信号阈值
int actuatorDuration = 3000;  // 执行器持续时间

void setup() {
    Serial.begin(9600);

    // 初始化各模块
    pneumatic.init(PUMP_PIN, VALVE_PIN);
    flex.init(A0);
    imu.init();
    emg.init(A1);
    unity.init();

    Serial.println("System ready");
}

// 临时测试用
#define TEST_MODE
// #define TEST_IMU  // 测试 IMU 时注释上面，开启这个

void loop() {
#ifdef TEST_IMU
    // 只测试 IMU，不动气动
    Serial.print("Yaw: "); Serial.println(imu.getYaw());
    Serial.print("Pitch: "); Serial.println(imu.getPitch());
    Serial.print("Roll: "); Serial.println(imu.getRoll());
    delay(100);
#endif

#ifdef TEST_MODE
    pneumatic.test();
    while (true) { }  // 跑完停止
#endif

#ifndef TEST_MODE
    // 正式状态机
    switch (currentState) {
        case STATE_IDLE:
            // 等待 Unity 发送指令
            if (unity.hasCommand()) {
                String cmd = unity.getCommand();
                if (cmd == "START") {
                    currentState = STATE_POSE_CHECK;
                    stateStartTime = millis();
                    Serial.println("Start pose check");
                }
            }
            break;

        case STATE_POSE_CHECK:
            // 姿态判定（IMU 或 弯曲传感器）
            // if (flex.getAngle() > poseThreshold || imu.getPitch() > poseThreshold) {
            //     unity.sendData("POSE_OK");
            //     currentState = STATE_BIO_MONITOR;
            // }
            currentState = STATE_BIO_MONITOR;  // TODO: 根据实际判定
            break;

        case STATE_BIO_MONITOR:
            // 生理数据监测（EMG/PPG）
            // int emgValue = emg.getValue();
            // unity.sendBioData(emgValue);
            // if (emgValue > bioThreshold) {
            //     currentState = STATE_ACTUATOR;
            // }
            currentState = STATE_ACTUATOR;  // TODO: 根据实际判定
            break;

        case STATE_ACTUATOR:
            // 执行器控制（气动 + 震动）
            if (millis() - stateStartTime < actuatorDuration) {
                pneumatic.inflate();
                // digitalWrite(VIBRATE_PIN, HIGH);
            } else {
                pneumatic.stop();
                // digitalWrite(VIBRATE_PIN, LOW);
                currentState = STATE_FEEDBACK;
                stateStartTime = millis();
            }
            break;

        case STATE_FEEDBACK:
            // 发送反馈给 Unity
            unity.sendData("ACTUATOR_DONE");
            currentState = STATE_IDLE;
            break;
    }
}