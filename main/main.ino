/**
 * @file main.ino
 * @brief 智能硬件主程序 - 精简版（只保留已完成的模块）
 * @desc 支持分模块测试，逐步集成
 */

// 已完成的模块
#include "ActuatorPneumatic.h"
#include "SensorFlex.h"
#include "SensorEMG.h"
#include "SensorPPG.h"

// ===== 引脚定义 =====
#define PUMP_PIN     8
#define VALVE_PIN    9
#define FLEX_PIN    A3
#define EMG_CH0    A0   // 通道0 - 大臂
#define EMG_CH1    A1   // 通道1 - 小臂
#define EMG_CH2    A2   // 通道2 - 手
#define PPG_PIN     A0   // PPG 传感器

// ===== 模块实例 =====
ActuatorPneumatic pneumatic;
SensorFlex flex;
SensorEMG emg;
SensorPPG ppg;

// ===== 测试模式选择 =====
#define TEST_FLEX
// #define TEST_EMG
// #define TEST_PPG
// #define TEST_PNEUMATIC
// #define TEST_ALL

// ===== 状态变量 =====
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100;

void setup() {
    Serial.begin(115200);

    #ifdef TEST_FLEX
        Serial.println("=== Test Flex ===");
        flex.init(FLEX_PIN);
    #endif

    #ifdef TEST_EMG
        Serial.println("=== Test EMG ===");
        emg.init(EMG_CH0, EMG_CH1, EMG_CH2);
    #endif

    #ifdef TEST_PPG
        Serial.println("=== Test PPG ===");
        ppg.init();
    #endif

    #ifdef TEST_PNEUMATIC
        Serial.println("=== Test Pneumatic ===");
        pneumatic.init(PUMP_PIN, VALVE_PIN);
    #endif

    #if defined(TEST_FLEX) || defined(TEST_EMG) || defined(TEST_PPG)
        Serial.println("f -> 校准伸直  b -> 校准弯曲  s -> 状态  r -> 重置");
    #endif

    Serial.println("Ready");
}

void loop() {
    unsigned long now = millis();

    // ===== Flex 测试 =====
    #ifdef TEST_FLEX
        flex.update();

        // 命令处理
        if (Serial.available()) {
            char cmd = Serial.read();
            if (cmd == 'f') flex.calibrateFlat();
            else if (cmd == 'b') flex.calibrateBent();
            else if (cmd == 's') flex.printStatus();
            else if (cmd == 'r') flex.reset();
        }

        // 定期输出
        if (now - lastPrintTime > PRINT_INTERVAL) {
            lastPrintTime = now;
            Serial.print("Flex - Angle: ");
            Serial.print(flex.getAngle(), 1);
            Serial.print("  State: ");
            switch (flex.getState()) {
                case FLEX_FLAT: Serial.print("FLAT"); break;
                case FLEX_MIDDLE: Serial.print("MIDDLE"); break;
                case FLEX_BENT: Serial.print("BENT"); break;
            }
            Serial.println();
        }
    #endif

    // ===== EMG 测试 =====
    #ifdef TEST_EMG
        emg.update();

        if (Serial.available()) {
            char cmd = Serial.read();
            if (cmd == 's') {
                Serial.println("=== EMG Status ===");
                for (int i = 0; i < 3; i++) {
                    Serial.print("CH");
                    Serial.print(i);
                    Serial.print(": ");
                    Serial.println(emg.getValue(i));
                }
            }
        }

        if (now - lastPrintTime > PRINT_INTERVAL) {
            lastPrintTime = now;
            Serial.print("EMG: ");
            Serial.print(emg.getValue(0));
            Serial.print(" ");
            Serial.print(emg.getValue(1));
            Serial.print(" ");
            Serial.println(emg.getValue(2));
        }
    #endif

    // ===== PPG 测试 =====
    #ifdef TEST_PPG
        ppg.update();

        if (now - lastPrintTime > PRINT_INTERVAL) {
            lastPrintTime = now;
            Serial.print("PPG - Raw: ");
            Serial.print(ppg.getRaw());
            Serial.print("  HR: ");
            Serial.println(ppg.getHeartRate());
        }
    #endif

    // ===== Pneumatic 测试 =====
    #ifdef TEST_PNEUMATIC
        pneumatic.update(500);  // 假设压力值

        if (Serial.available()) {
            char cmd = Serial.read();
            if (cmd == 'i') {
                Serial.println("Inflate");
                pneumatic.startInflate();
            }
            else if (cmd == 'd') {
                Serial.println("Deflate");
                pneumatic.startDeflate();
            }
            else if (cmd == 's') {
                Serial.println("Stop");
                pneumatic.stop();
            }
        }
    #endif

    delay(20);
}