// 主循环 - 分层架构（双IMU + TF卡版本）
// 命令层 → 传感器层 → 联动层 → 硬件层

#include "ActuatorPneumatic.h"
#include "ActuatorVibration.h"
#include "SensorIMU.h"
#include "SensorFlex.h"
#include "SensorPressure.h"
#include "SensorTF.h"

// ===== 引脚定义 =====
#define PUMP_PIN 8
#define VALVE_PIN 9
#define MOTOR_PIN 6
const int FLEX_PIN = A3;
const int PRESSURE_PIN = A0;
const int TF_CS_PIN = 10;

// ===== IMU地址 =====
const uint8_t IMU1_ADDR = 0x68;
const uint8_t IMU2_ADDR = 0x69;

// ===== 打印间隔 =====
const unsigned long PRINT_INTERVAL = 300;
const unsigned long LOG_INTERVAL = 100;

// ===== 模块实例 =====
ActuatorPneumatic pneumatic;
ActuatorVibration vibration;
SensorIMU imuUpper(IMU1_ADDR, "IMU1");   // 上臂
SensorIMU imuLower(IMU2_ADDR, "IMU2");   // 前臂
DualIMUPostureClassifier postureClassifier;
SensorFlex flex(FLEX_PIN);
SensorPressure pressure(PRESSURE_PIN);
SensorTF tfLogger;

// ===== 配置开关 =====
bool enableFlexPneumatic = true;
bool enableVibration = true;
bool enableTFLogging = true;
bool enableDualIMUPrint = true;

// ===== 时间变量 =====
unsigned long lastIMUPrintTime = 0;
unsigned long lastLogTime = 0;

// ===== 姿态状态 =====
Posture combinedPosture = POSTURE_UNKNOWN;

// ===== 气动联动变量 =====
static unsigned long inflateStartTime = 0;
static bool isInflating = false;
static bool inflateTriggered = false;

// ===== 震动联动变量 =====
static unsigned long vibrationDelayStart = 0;
static bool vibrationDelayTriggered = false;
static bool lastPressureState = false;
static bool vibrationPatternActive = false;

// ===== TF日志事件 =====
void logTFEvent(const __FlashStringHelper *eventText) {
    if (!enableTFLogging || !tfLogger.isReady()) {
        return;
    }
    tfLogger.appendEvent(millis(), eventText);
}

// ===== 打印双IMU数据 =====
void printDualIMUData() {
    imuUpper.printLabeledData();
    imuLower.printLabeledData();
    imuUpper.printArmState();
    imuLower.printArmState();
    Serial.print(F("[COMBINED] Posture: "));
    Serial.print(postureToString(combinedPosture));
    Serial.print(F(" | Score: "));
    Serial.println(postureClassifier.getCurrentScore(), 4);
}

// ============================================================
// 命令层：handleCommand()
// ============================================================
void handleCommand() {
    if (!Serial.available()) {
        return;
    }

    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') {
        return;
    }

    switch (cmd) {
        case 'f':
            flex.calibrateFlat();
            Serial.println(F(">> Flex flat calibration done"));
            logTFEvent(F("EVENT:flex_flat_calibrated"));
            break;

        case 'b':
            flex.calibrateBent();
            Serial.println(F(">> Flex bent calibration done"));
            logTFEvent(F("EVENT:flex_bent_calibrated"));
            break;

        case 's':
            flex.printStatus();
            break;

        case 'r':
            flex.reset();
            Serial.println(F(">> Flex calibration reset"));
            logTFEvent(F("EVENT:flex_reset"));
            break;

        case 'i':
            Serial.println(F("[Test] Read both IMUs once"));
            imuUpper.update();
            imuLower.update();
            combinedPosture = postureClassifier.update(imuUpper, imuLower);
            printDualIMUData();
            break;

        case 'o':
            enableDualIMUPrint = !enableDualIMUPrint;
            Serial.print(F("[Config] Dual IMU serial output "));
            Serial.println(enableDualIMUPrint ? F("enabled") : F("disabled"));
            break;

        case 't':
            Serial.println(F("[Test] Pneumatic inflate for 3 seconds"));
            pneumatic.startInflate();
            delay(3000);
            pneumatic.stop();
            Serial.println(F("[Test] Pneumatic test done"));
            logTFEvent(F("EVENT:pneumatic_test"));
            break;

        case 'v':
            Serial.println(F("[Test] Pressure sensor"));
            pressure.test();
            break;

        case 'w':
            Serial.println(F("[Test] Vibration motor"));
            vibration.start();
            logTFEvent(F("EVENT:vibration_test"));
            break;

        case 'e':
            enableFlexPneumatic = true;
            Serial.println(F("[Config] Flex to pneumatic enabled"));
            logTFEvent(F("EVENT:flex_pneumatic_enabled"));
            break;

        case 'd':
            enableFlexPneumatic = false;
            Serial.println(F("[Config] Flex to pneumatic disabled"));
            logTFEvent(F("EVENT:flex_pneumatic_disabled"));
            break;

        case 'm':
            enableTFLogging = !enableTFLogging;
            Serial.print(F("[Config] TF logging "));
            Serial.println(enableTFLogging ? F("enabled") : F("disabled"));
            if (enableTFLogging) {
                logTFEvent(F("EVENT:tf_logging_enabled"));
            }
            break;

        case 'l':
            Serial.print(F("[TF] ready="));
            Serial.print(tfLogger.isReady() ? F("yes") : F("no"));
            Serial.print(F(", file="));
            Serial.println(tfLogger.getFileName());
            break;

        case 'h':
            Serial.println(F("===== Command Help ====="));
            Serial.println(F("f -> calibrate flex flat"));
            Serial.println(F("b -> calibrate flex bent"));
            Serial.println(F("s -> show flex status"));
            Serial.println(F("r -> reset flex calibration"));
            Serial.println(F("i -> print both IMUs once"));
            Serial.println(F("o -> toggle periodic dual-IMU output"));
            Serial.println(F("t -> test pneumatic"));
            Serial.println(F("v -> test pressure sensor"));
            Serial.println(F("w -> test vibration motor"));
            Serial.println(F("e -> enable flex pneumatic"));
            Serial.println(F("d -> disable flex pneumatic"));
            Serial.println(F("m -> toggle TF logging"));
            Serial.println(F("l -> show TF logger status"));
            Serial.println(F("h -> help"));
            break;
    }
}

// ============================================================
// 传感器层
// ============================================================
void updateFlex() {
    if (!flex.isCalibrated()) {
        static unsigned long lastReminder = 0;
        if (millis() - lastReminder >= 3000) {
            lastReminder = millis();
            Serial.println(F(">> Please calibrate flex sensor: f then b"));
        }
        return;
    }

    flex.update();
}

void updateIMUs() {
    imuUpper.update();
    imuLower.update();
    combinedPosture = postureClassifier.update(imuUpper, imuLower);

    unsigned long now = millis();
    if (enableDualIMUPrint && now - lastIMUPrintTime >= PRINT_INTERVAL) {
        lastIMUPrintTime = now;
        printDualIMUData();
    }
}

// ============================================================
// 联动层：controlPneumatic()
// 充气条件：弯曲伸直 + (IMU半抬 或 IMU举起)
// 放气条件：弯曲传感器弯曲 → 立即放气
// ============================================================
void controlPneumatic() {
    if (!enableFlexPneumatic || !flex.isCalibrated()) {
        return;
    }

    FlexDetailedState fState = flex.getDetailedState();
    ArmState imuState = combinedPosture;

    bool shouldInflate = false;
    if (fState == FLEX_DETAILED_FLAT && imuState == ARM_STATE_HALF_RAISED) {
        shouldInflate = true;
    } else if (fState == FLEX_DETAILED_FLAT && imuState == ARM_STATE_PICKING) {
        shouldInflate = true;
    }

    bool shouldDeflate = (fState == FLEX_DETAILED_BENT);

    if (shouldInflate && !isInflating && !inflateTriggered) {
        pneumatic.startInflate();
        inflateStartTime = millis();
        isInflating = true;
        inflateTriggered = true;
        logTFEvent(F("EVENT:pneumatic_inflate_start"));
    }

    if (isInflating && millis() - inflateStartTime >= 5000) {
        pneumatic.stop();
        isInflating = false;
        logTFEvent(F("EVENT:pneumatic_stop"));
    }

    if (shouldDeflate && (isInflating || inflateTriggered)) {
        pneumatic.stop();
        pneumatic.startDeflate();
        isInflating = false;
        inflateTriggered = false;
        logTFEvent(F("EVENT:pneumatic_deflate"));
    }
}

// 检查是否可以重新触发充气
void resetPneumaticIfNeeded() {
    FlexDetailedState fState = flex.getDetailedState();
    ArmState imuState = combinedPosture;

    if (fState == FLEX_DETAILED_BENT && imuState == ARM_STATE_HALF_RAISED) {
        inflateTriggered = false;
    }
}

// ============================================================
// 联动层：controlVibration()
// 触发条件（三选一，延迟2秒）：
//   1. 弯曲传感器伸直 + IMU半抬
//   2. IMU举起 + 弯曲传感器未伸直
//   3. 两者都到位 + 压力传感器未按下
// 压力传感器按下：停止本次震动
// ============================================================
void controlVibration() {
    if (!enableVibration || !flex.isCalibrated() || !flex.isCalibrationValid()) {
        return;
    }

    FlexDetailedState fState = flex.getDetailedState();
    ArmState imuState = combinedPosture;
    bool pressurePressed = pressure.isPressed();

    // 压力传感器按下：立即停止震动
    if (pressurePressed) {
        if (vibration.isActive()) {
            vibration.stop();
            logTFEvent(F("EVENT:vibration_stop_pressure"));
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

    // 情况1: 弯曲传感器伸直 + IMU半抬
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
    if (millis() - vibrationDelayStart >= 2000 && !vibrationPatternActive) {
        vibrationPatternActive = true;
        vibration.setDuration(500);
        vibration.start();
        logTFEvent(F("EVENT:vibration_start"));
    }
}

// ============================================================
// 日志层：logSensorData()
// ============================================================
void logSensorData() {
    if (!enableTFLogging || !tfLogger.isReady()) {
        return;
    }

    unsigned long now = millis();
    if (now - lastLogTime < LOG_INTERVAL) {
        return;
    }
    lastLogTime = now;

    tfLogger.appendSample(
        now,
        imuUpper.getAccX(),
        imuUpper.getAccY(),
        imuUpper.getAccZ(),
        imuUpper.getGyroX(),
        imuUpper.getGyroY(),
        imuUpper.getGyroZ(),
        (int)imuUpper.getArmState(),
        imuLower.getAccX(),
        imuLower.getAccY(),
        imuLower.getAccZ(),
        imuLower.getGyroX(),
        imuLower.getGyroY(),
        imuLower.getGyroZ(),
        (int)imuLower.getArmState(),
        (int)combinedPosture,
        flex.getRaw(),
        flex.getAngle(),
        (int)flex.getDetailedState(),
        pressure.getRaw(),
        pressure.isPressed(),
        enableFlexPneumatic,
        enableVibration,
        isInflating,
        inflateTriggered,
        vibration.isActive()
    );
}

// ============================================================
// 主程序
// ============================================================
void setup() {
    Serial.begin(115200);

    pneumatic.init(PUMP_PIN, VALVE_PIN);
    vibration.init(MOTOR_PIN);
    flex.init();
    pressure.init();

    // 配置姿态分类器
    postureClassifier.setStableThreshold(3);
    postureClassifier.setSwitchMargin(0.08f);

    // 初始化双IMU
    bool imu1Ok = imuUpper.begin();
    bool imu2Ok = imuLower.begin();

    if (imu1Ok) {
        Serial.println(F("[IMU1] init OK at 0x68"));
    } else {
        Serial.println(F("[IMU1] init failed at 0x68"));
    }

    if (imu2Ok) {
        Serial.println(F("[IMU2] init OK at 0x69"));
    } else {
        Serial.println(F("[IMU2] init failed at 0x69"));
    }

    // 初始化TF卡
    if (!tfLogger.begin(TF_CS_PIN, "imu_log.txt")) {
        Serial.println(F("[TF] init failed, logging disabled"));
        enableTFLogging = false;
    } else {
        Serial.print(F("[TF] logging to "));
        Serial.println(tfLogger.getFileName());
        logTFEvent(F("EVENT:system_start_dual_imu"));
    }

    Serial.println(F("=== System Ready ==="));
    Serial.println(F("Two IMUs share I2C bus: IMU1=0x68, IMU2=0x69"));
    Serial.println(F("Combined posture uses both IMUs with stable switching."));
    Serial.println(F("Use command i to inspect both IMUs, o to toggle streaming."));
    Serial.println(F("h -> View command help"));
}

void loop() {
    handleCommand();          // 命令层
    updateIMUs();             // 传感器层：双IMU更新
    updateFlex();             // 传感器层
    pressure.update();        // 传感器层
    controlPneumatic();       // 联动层
    resetPneumaticIfNeeded(); // 联动层
    controlVibration();       // 联动层：震动控制
    vibration.update();       // 震动计时控制
    logSensorData();          // 日志层
}