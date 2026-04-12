// 主循环 - 分层架构
// 命令层 → 传感器层 → 联动层 → 硬件层

#include "ActuatorPneumatic.h"
#include "SensorIMU.h"
#include "SensorPPG.h"
#include "SensorFlex.h"

// ===== 引脚定义 =====
#define PUMP_PIN 8
#define VALVE_PIN 9
const int FLEX_PIN = A3;
const int PPG_PIN = A0;

// ===== 模块实例 =====
ActuatorPneumatic pneumatic;
SensorIMU imu;
SensorPPG ppg(PPG_PIN, 50);
SensorFlex flex(FLEX_PIN);

// ===== 联动配置 =====
bool enableFlexPneumatic = true;  // 弯曲→气动联动

// ============================================================
// 命令层：handleCommand()
// ============================================================
void handleCommand() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') return;

    // 校准命令
    switch (cmd) {
        case 'f':
            flex.calibrateFlat();
            Serial.println(">> 伸直校准完成");
            break;
        case 'b':
            flex.calibrateBent();
            Serial.print(">> 弯曲校准完成, Diff=");
            Serial.println(abs(flex.getRaw() - 0));  // TODO: 获取校准值
            break;
        case 's':
            flex.printStatus();
            break;
        case 'r':
            flex.reset();
            Serial.println(">> 已重置校准");
            break;
        case 't':
            Serial.println("[测试] 气动充气 3秒...");
            pneumatic.startInflate();
            delay(3000);
            pneumatic.stop();
            Serial.println("[测试] 完成");
            break;
        case 'p':
            Serial.println("[测试] PPG 输出...");
            ppg.update();
            ppg.test();
            break;
        case 'i':
            Serial.println("[测试] IMU...");
            imu.update();
            Serial.print("Pitch: ");
            Serial.println(imu.getPitch());
            break;
        case 'e':
            enableFlexPneumatic = true;
            Serial.println("[配置] 弯曲→气动: 已启用");
            break;
        case 'd':
            enableFlexPneumatic = false;
            Serial.println("[配置] 弯曲→气动: 已禁用");
            break;
        case 'h':
            Serial.println("===== 命令帮助 =====");
            Serial.println("f -> 校准伸直");
            Serial.println("b -> 校准弯曲");
            Serial.println("s -> 状态");
            Serial.println("r -> 重置");
            Serial.println("t -> 测试气动");
            Serial.println("p -> 测试PPG");
            Serial.println("i -> 测试IMU");
            Serial.println("e -> 启用弯曲→气动");
            Serial.println("d -> 禁用弯曲→气动");
            Serial.println("h -> 帮助");
            break;
    }
}

// ============================================================
// 传感器层
// ============================================================
unsigned long lastPPGPrintTime = 0;
const unsigned long PRINT_INTERVAL = 500;

void updateFlex() {
    flex.update();
}

void updatePPG() {
    ppg.update();

    unsigned long now = millis();
    if (now - lastPPGPrintTime >= PRINT_INTERVAL) {
        lastPPGPrintTime = now;
        ppg.test();
    }

    delay(20);
}

// ============================================================
// 联动层：controlPneumatic()
// ============================================================
static unsigned long inflateStartTime = 0;
static bool isInflating = false;
static bool wasFlat = false;
static bool justInflated = false;

void controlPneumatic() {
    if (!enableFlexPneumatic) return;
    if (!flex.isCalibrated()) return;

    // 使用 SensorFlex 的状态判断
    // bentTriggered 在 SensorFlex 里实际是"弯曲触发"
    // 但我们的逻辑是伸直触发，需要转换
    //
    // SensorFlex 状态：FLEX_FLAT(伸直), FLEX_MIDDLE, FLEX_BENT(弯曲)
    // 我们需要：伸直 → 充气，弯曲 → 放气

    FlexState state = flex.getState();
    bool isFlat = (state == FLEX_FLAT);  // 伸直状态

    if (isFlat) {
        // 伸直 → 充气 4 秒（只充一次，需弯曲后才重置）
        if (!isInflating && !justInflated) {
            pneumatic.startInflate();
            inflateStartTime = millis();
            isInflating = true;
            wasFlat = true;
            justInflated = true;
            Serial.println("[气动] 伸直 → 开始充气");
        }
        // 充气 4 秒后停止
        if (isInflating && millis() - inflateStartTime >= 4000) {
            pneumatic.stop();
            isInflating = false;
            Serial.println("[气动] 充气完成 (4s)");
        }
    } else {
        // 弯曲 → 放气
        if (isInflating || wasFlat) {
            pneumatic.stop();
            pneumatic.startDeflate();
            isInflating = false;
            wasFlat = false;
            justInflated = false;
            Serial.println("[气动] 弯曲 → 放气");
        }
    }
}

// ============================================================
// 主程序
// ============================================================
void setup() {
    Serial.begin(115200);

    pneumatic.init(PUMP_PIN, VALVE_PIN);
    flex.init();
    imu.init();
    ppg.init();
    imu.requestEuler();

    Serial.println("=== 系统就绪 ===");
    Serial.println("h -> 查看命令帮助");
}

void loop() {
    handleCommand();      // 命令层
    updateFlex();         // 传感器层
    updatePPG();          // 传感器层
    controlPneumatic();   // 联动层
}