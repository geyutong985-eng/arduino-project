// 主循环 - 分层架构
// 命令层 → 传感器层 → 联动层 → 硬件层

#include "ActuatorPneumatic.h"
#include "ActuatorVibration.h"
#include "SensorIMU.h"
// #include "SensorPPG.h"  // 禁用以节省内存
#include "SensorFlex.h"
#include "SensorPressure.h"

// ===== 引脚定义 =====
#define PUMP_PIN 8
#define VALVE_PIN 9
#define MOTOR_PIN 6
const int FLEX_PIN = A3;
// const int PPG_PIN = A0;
const int PRESSURE_PIN = A4;

// ===== 模块实例 =====
ActuatorPneumatic pneumatic;
ActuatorVibration vibration;
SensorIMU imu;
// SensorPPG ppg(PPG_PIN, 50);  // 禁用
SensorFlex flex(FLEX_PIN);
SensorPressure pressure(PRESSURE_PIN);

// ===== 联动配置 =====
bool enableFlexPneumatic = true;  // 弯曲→气动联动
bool enableVibration = true;     // 震动触发

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
        // case 'p':
            // Serial.println("[测试] PPG 输出...");
            // ppg.update();
            // ppg.test();
            // break;
        case 'i':
            Serial.println("[测试] IMU...");
            imu.update();
            Serial.print("Pitch: ");
            Serial.println(imu.getPitch());
            break;
        case 'v':
            Serial.println("[测试] 压力传感器...");
            pressure.test();
            break;
        case 'w':
            Serial.println("[测试] 震动马达...");
            vibration.start();
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
            Serial.println("v -> 测试压力传感器");
            Serial.println("w -> 测试震动马达");
            Serial.println("h -> 帮助");
            break;
    }
}

// ============================================================
// 传感器层
// ============================================================
unsigned long lastIMUPrintTime = 0;
const unsigned long PRINT_INTERVAL = 500;

void updateFlex() {
    if (!flex.isCalibrated()) {
        static unsigned long lastReminder = 0;
        if (millis() - lastReminder >= 3000) {
            lastReminder = millis();
            Serial.println(">> 请校准弯曲传感器: f (伸直) + b (弯曲)");
        }
        return;
    }
    flex.update();

    // 调试：显示详细状态
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug >= 2000) {
        lastDebug = millis();
        int raw = flex.getRaw();
        // 注意：这里无法直接访问 private 成员，需要添加 getter
    }
}

// void updatePPG() {
//     ppg.update();

    // unsigned long now = millis();
    // if (now - lastPPGPrintTime >= PRINT_INTERVAL) {
    //     lastPPGPrintTime = now;
    //     ppg.test();
    // }
// }

void updateIMU() {
    imu.update();

    unsigned long now = millis();
    if (now - lastIMUPrintTime >= PRINT_INTERVAL) {
        lastIMUPrintTime = now;
        imu.test();
        imu.printArmState();
    }
}

// ============================================================
// 联动层：controlPneumatic()
// 充气条件（4种任一满足 → 充气5秒）：
//   1. 弯曲中间 + IMU半抬
//   2. 弯曲中间 + IMU举起
//   3. 弯曲伸直 + IMU半抬
//   4. 弯曲伸直 + IMU举起
// 放气条件：弯曲传感器弯曲 → 立即放气
// ============================================================
static unsigned long inflateStartTime = 0;
static bool isInflating = false;
static bool inflateTriggered = false;  // 充气已触发过

void controlPneumatic() {
    if (!enableFlexPneumatic) return;
    if (!flex.isCalibrated()) return;

    FlexDetailedState fState = flex.getDetailedState();
    ArmState imuState = imu.getArmState();

    // 判断是否应该充气（两种条件：伸直+半抬 或 伸直+举起）
    bool shouldInflate = false;

    if (fState == FLEX_DETAILED_FLAT && imuState == ARM_STATE_HALF_RAISED) {
        shouldInflate = true;
    } else if (fState == FLEX_DETAILED_FLAT && imuState == ARM_STATE_PICKING) {
        shouldInflate = true;
    }

    // 判断是否应该放气（弯曲即放）
    bool shouldDeflate = (fState == FLEX_DETAILED_BENT);

    // 充气逻辑
    if (shouldInflate && !isInflating && !inflateTriggered) {
        pneumatic.startInflate();
        inflateStartTime = millis();
        isInflating = true;
        inflateTriggered = true;
    }

    // 充气5秒后停止
    if (isInflating && millis() - inflateStartTime >= 5000) {
        pneumatic.stop();
        isInflating = false;
    }

    // 放气逻辑（弯曲即放）
    if (shouldDeflate && (isInflating || inflateTriggered)) {
        pneumatic.stop();
        pneumatic.startDeflate();
        isInflating = false;
        inflateTriggered = false;
    }
}

// 检查是否可以重新触发充气（状态回到初始时）
void resetPneumaticIfNeeded() {
    FlexDetailedState fState = flex.getDetailedState();
    ArmState imuState = imu.getArmState();

    // 当状态回到初始（弯曲+半抬）时，可以重新触发充气
    if (fState == FLEX_DETAILED_BENT && imuState == ARM_STATE_HALF_RAISED) {
        inflateTriggered = false;
    }
}

// ============================================================
// 联动层：controlVibration()
// 触发条件（三选一，延迟2秒）：
//   1. 弯曲传感器伸直 + IMU未到举起
//   2. IMU举起 + 弯曲传感器未伸直
//   3. 两者都到位 + 压力传感器未按下
// 震动模式：震动500ms → 停5秒 → 再震动 → 重复
// 压力传感器按下：停止本次震动；松开后重新监测
// ============================================================
static unsigned long vibrationDelayStart = 0;
static bool vibrationDelayTriggered = false;
static bool lastPressureState = false;  // 上次压力传感器状态
static unsigned long vibrationRepeatStart = 0;
static bool vibrationPatternActive = false;

void controlVibration() {
    if (!enableVibration) return;
    if (!flex.isCalibrated()) return;
    if (!flex.isCalibrationValid()) return;

    FlexDetailedState fState = flex.getDetailedState();
    ArmState imuState = imu.getArmState();
    bool pressurePressed = pressure.isPressed();

    // 压力传感器按下：立即停止震动
    if (pressurePressed) {
        if (vibration.isActive()) {
            vibration.stop();
        }
        vibrationDelayTriggered = false;
        vibrationPatternActive = false;
        lastPressureState = true;
        return;
    }

    // 检测压力传感器从按下到松开的转变
    if (lastPressureState && !pressurePressed) {
        vibrationDelayTriggered = false;
    }
    lastPressureState = pressurePressed;

    // 判断是否应该触发震动
    bool shouldVibrate = false;

    // 情况1: 弯曲传感器伸直 + IMU半抬（排除自然下垂）
    if (fState == FLEX_DETAILED_FLAT && imuState == ARM_STATE_HALF_RAISED) {
        shouldVibrate = true;
    }
    // 情况2: IMU举起 + 弯曲传感器未伸直
    else if (imuState == ARM_STATE_PICKING && fState != FLEX_DETAILED_FLAT) {
        shouldVibrate = true;
    }
    // 情况3: 两者都到位 + 压力传感器未按下
    else if (fState == FLEX_DETAILED_FLAT && imuState == ARM_STATE_PICKING && !pressurePressed) {
        shouldVibrate = true;
    }

    if (!shouldVibrate) {
        vibrationDelayTriggered = false;
        vibrationPatternActive = false;
        return;
    }

    // 触发震动
    if (!vibrationDelayTriggered) {
        vibrationDelayStart = millis();
        vibrationDelayTriggered = true;
        vibrationPatternActive = false;
    }

    // 2秒延时后开始震动模式
    if (millis() - vibrationDelayStart >= 2000) {
        if (!vibrationPatternActive) {
            vibrationPatternActive = true;
            vibrationRepeatStart = millis();
            vibration.setDuration(500);
            vibration.start();
        }
    }
}

// ============================================================
// 主程序
// ============================================================
void setup() {
    Serial.begin(115200);

    pneumatic.init(PUMP_PIN, VALVE_PIN);
    vibration.init(MOTOR_PIN);
    flex.init();
    imu.init();
    // ppg.init();
    pressure.init();
    pressure.setThreshold(150);
    imu.requestEuler();

    Serial.println("=== 系统就绪 ===");
    Serial.println("h -> 查看命令帮助");
}

void loop() {
    handleCommand();       // 命令层
    updateIMU();            // IMU更新+打印
    imu.detectPickAction(); // 手势识别
    updateFlex();           // 传感器层
    // updatePPG();          // 传感器层 (已禁用)
    pressure.update();      // 传感器层
    controlPneumatic();     // 联动层
    controlVibration();     // 联动层：震动控制
    vibration.update();     // 震动计时控制
}