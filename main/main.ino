// 主循环 - 分层架构（双IMU + 双气动）
// 命令层 → 传感器层 → 联动层 → 硬件层

#include "ActuatorPneumatic.h"
#include "ActuatorPneumatic2.h"
#include "ActuatorVibration.h"
#include "SensorIMU.h"
#include "SensorFlex.h"
#include "SensorPressure.h"
#include "PneumaticState.h"
#include "MadgwickRelativeAngle.h"
#include "DTWMotionClassifier.h"

#if defined(ESP32)
#include <Esp.h>
#endif

// ===== ESP32 引脚定义 =====
#define PALM_PUMP_PIN 18
#define PALM_VALVE_PIN 19
#define FOREARM_PUMP_PIN 4
#define FOREARM_VALVE_PIN 5
#define MOTOR_PIN 2
#define MOTOR2_PIN 15
const int FLEX_PIN = 34;
const int PRESSURE_PIN = 35;

// ===== IMU I2C 引脚 (固定) =====
const int IMU_SDA_PIN = 21;
const int IMU_SCL_PIN = 22;

// ===== IMU地址 =====
const uint8_t IMU1_ADDR = 0x68;
const uint8_t IMU2_ADDR = 0x69;

// ===== 打印间隔 =====
const unsigned long PRINT_INTERVAL = 500;
const unsigned long DTW_SAMPLE_INTERVAL = 50;
const unsigned long DTW_RECHECK_INTERVAL = 400;
const unsigned long DTW_CONFIRM_HOLD_MS = 1200;
const uint8_t DTW_MIN_CAPTURE_FRAMES = 8;

// ===== 模块实例 =====
ActuatorPneumatic pneumatic;      // 手掌气动
ActuatorPneumatic2 pneumatic2;  // 小臂气动
ActuatorVibration vibration;    // GPIO2 震动
ActuatorVibration vibration2;  // GPIO15 震动
SensorIMU imuUpper(IMU1_ADDR, "IMU1");   // 上臂
SensorIMU imuLower(IMU2_ADDR, "IMU2");   // 前臂
DualIMUPostureClassifier postureClassifier;
SensorFlex flex(FLEX_PIN);
SensorPressure pressure(PRESSURE_PIN);
MadgwickRelativeAngle upperAngleFilter;
MadgwickRelativeAngle forearmAngleFilter;
DTWMotionClassifier dtwClassifier;

// ===== 配置开关 =====
bool enablePalmPneumatic = true;
bool enableForearmPneumatic = true;
bool enableVibration = true;
bool enableDualIMUPrint = true;
bool enableDtwPrint = true;

// ===== 时间变量 =====
unsigned long lastIMUPrintTime = 0;
unsigned long lastDtwSampleTime = 0;
unsigned long lastDtwEvalTime = 0;
unsigned long lastDtwConfirmTime = 0;
unsigned long lastDtwFilterTime = 0;

// ===== 姿态状态 =====
Posture combinedPosture = POSTURE_UNKNOWN;
Posture dtwConfirmedPosture = POSTURE_UNKNOWN;
bool dtwNeutralCaptured = false;

// ===== 小臂气动联动变量 =====
static bool forearmInflateTriggered = false;
static bool lastPressurePressed = false;

// ===== 震动联动变量 =====
// 记录上一个IMU状态，用于检测状态变化
static Posture lastPostureState = POSTURE_UNKNOWN;

// D10震动：检测到Half Raised变化后3秒未到Picking触发
static unsigned long halfRaisedTime = 0;
static bool halfRaisedTriggered = false;

// D6震动：检测到Picking变化后2秒未按压触发
static unsigned long pickingTime = 0;
static bool pickingTriggered = false;

// 通用
static bool lastPressureState = false;

// TF卡已移除，保留空函数兼容现有事件调用
void logTFEvent(const __FlashStringHelper *eventText) {}
ArmState getControlArmState();

// ===== 打印 ESP32 内存情况 =====
void printMemoryInfo() {
#if defined(ESP32)
    Serial.print(F("[MEMORY] Free heap: "));
    Serial.print(ESP.getFreeHeap());
    Serial.println(F(" bytes"));
#else
    Serial.println(F("[MEMORY] ESP32 heap info unavailable on this board"));
#endif

    // 静态对象内存估算
    size_t staticSize =
        sizeof(ActuatorPneumatic) + sizeof(ActuatorPneumatic2) +
        sizeof(ActuatorVibration) * 2 + sizeof(SensorIMU) * 2 +
        sizeof(DualIMUPostureClassifier) + sizeof(SensorFlex) +
        sizeof(SensorPressure) + sizeof(DTWMotionClassifier) +
        sizeof(MadgwickRelativeAngle) * 2;

    Serial.print(F("[MEMORY] Static objects: ~"));
    Serial.print(staticSize);
    Serial.println(F(" bytes"));
}

// ===== 打印双IMU数据 =====
void printDualIMUData() {
    imuUpper.printLabeledData();
    imuLower.printLabeledData();
    Serial.print(F("[COMBINED] Posture: "));
    Serial.print(postureToString(combinedPosture));
    Serial.print(F(" | Score: "));
    Serial.print(postureClassifier.getCurrentScore(), 4);
    Serial.print(F(" | DTW: "));
    Serial.println(postureToString(getControlArmState()));
}

uint8_t postureToDtwAction(Posture posture) {
    switch (posture) {
        case POSTURE_HALF_RAISED:
            return DTW_ACTION_HALF_RAISED;
        case POSTURE_PICKING:
            return DTW_ACTION_PICKING;
        default:
            return DTW_ACTION_NONE;
    }
}

Posture dtwActionToPosture(uint8_t actionId) {
    switch (actionId) {
        case DTW_ACTION_HALF_RAISED:
            return POSTURE_HALF_RAISED;
        case DTW_ACTION_PICKING:
            return POSTURE_PICKING;
        default:
            return POSTURE_UNKNOWN;
    }
}

ArmState getControlArmState() {
    if (dtwConfirmedPosture != POSTURE_UNKNOWN &&
        millis() - lastDtwConfirmTime <= DTW_CONFIRM_HOLD_MS) {
        return dtwConfirmedPosture;
    }
    return combinedPosture;
}

void captureDtwNeutral() {
    upperAngleFilter.captureNeutral();
    forearmAngleFilter.captureNeutral();
    dtwNeutralCaptured = true;
    Serial.println(F("[DTW] Neutral captured"));
}

void updateDtwFilters() {
    unsigned long now = millis();
    if (lastDtwFilterTime == 0) {
        lastDtwFilterTime = now;
        return;
    }

    float dtSeconds = (now - lastDtwFilterTime) / 1000.0f;
    lastDtwFilterTime = now;

    upperAngleFilter.update(
        imuUpper.getGyroX(), imuUpper.getGyroY(), imuUpper.getGyroZ(),
        imuUpper.getAccX(), imuUpper.getAccY(), imuUpper.getAccZ(),
        dtSeconds
    );
    forearmAngleFilter.update(
        imuLower.getGyroX(), imuLower.getGyroY(), imuLower.getGyroZ(),
        imuLower.getAccX(), imuLower.getAccY(), imuLower.getAccZ(),
        dtSeconds
    );
}

void buildDtwFeatureVector(float features[DTW_FEATURE_COUNT]) {
    float upperAngle = dtwNeutralCaptured ? upperAngleFilter.getRelativeAngleDeg() : 0.0f;
    float forearmAngle = dtwNeutralCaptured ? forearmAngleFilter.getRelativeAngleDeg() : 0.0f;
    float flexAngle = (flex.isCalibrated() && flex.isCalibrationValid()) ? flex.getAngle() : 0.0f;

    features[0] = imuUpper.getAccX();
    features[1] = imuUpper.getAccY();
    features[2] = imuUpper.getAccZ();
    features[3] = imuLower.getAccX();
    features[4] = imuLower.getAccY();
    features[5] = imuLower.getAccZ();
#if DTW_USE_RELATIVE_ANGLES
    features[6] = upperAngle / 90.0f;
    features[7] = forearmAngle / 90.0f;
#else
    features[6] = 0.0f;
    features[7] = 0.0f;
#endif
    features[8] = flexAngle / 90.0f;
}

void evaluateDtwCapture() {
    if (dtwClassifier.getCaptureCount() < DTW_MIN_CAPTURE_FRAMES) {
        return;
    }

    DtwMotionResult halfRaised = dtwClassifier.evaluate(DTW_ACTION_HALF_RAISED);
    DtwMotionResult picking = dtwClassifier.evaluate(DTW_ACTION_PICKING);
    DtwMotionResult best = {false, DTW_ACTION_NONE, 999999.0f, 0.0f};

    if (halfRaised.matched) {
        best = halfRaised;
    }
    if (picking.matched) {
        bool pickingIsBetter = !best.matched ||
            (picking.threshold > 0.0f && best.threshold > 0.0f &&
             (picking.distance / picking.threshold) < (best.distance / best.threshold));
        if (pickingIsBetter) {
            best = picking;
        }
    }

    if (enableDtwPrint) {
        Serial.print(F("[DTW] frames="));
        Serial.print(dtwClassifier.getCaptureCount());
        Serial.print(F(" half="));
        Serial.print(halfRaised.distance, 4);
        Serial.print('/');
        Serial.print(halfRaised.threshold, 4);
        Serial.print(halfRaised.matched ? F("*") : F(""));
        Serial.print(F(" picking="));
        Serial.print(picking.distance, 4);
        Serial.print('/');
        Serial.print(picking.threshold, 4);
        Serial.print(picking.matched ? F("*") : F(""));
        Serial.print(F(" best="));
        Serial.println(postureToString(dtwActionToPosture(best.actionId)));
    }

    if (best.matched) {
        dtwConfirmedPosture = dtwActionToPosture(best.actionId);
        lastDtwConfirmTime = millis();
    }
}

void updateDtwMotionCapture() {
    unsigned long now = millis();

    if (now - lastDtwSampleTime >= DTW_SAMPLE_INTERVAL) {
        lastDtwSampleTime = now;
        float features[DTW_FEATURE_COUNT];
        buildDtwFeatureVector(features);
        dtwClassifier.addSample(features);
    }

    if (dtwClassifier.getCaptureCount() >= DTW_MIN_CAPTURE_FRAMES &&
        now - lastDtwEvalTime >= DTW_RECHECK_INTERVAL) {
        lastDtwEvalTime = now;
        evaluateDtwCapture();
    }
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

        case 'm':
            captureDtwNeutral();
            logTFEvent(F("EVENT:dtw_neutral_captured"));
            break;

        case 'q':
            enableDtwPrint = !enableDtwPrint;
            Serial.print(F("[Config] DTW serial output "));
            Serial.println(enableDtwPrint ? F("enabled") : F("disabled"));
            break;

        case 't':
            Serial.println(F("[Test] Palm pneumatic inflate for 3 seconds"));
            pneumatic.startInflate();
            delay(3000);
            pneumatic.stop();
            Serial.println(F("[Test] Palm pneumatic test done"));
            logTFEvent(F("EVENT:palm_pneumatic_test"));
            break;

        case 'u':
            Serial.println(F("[Test] Forearm pneumatic inflate"));
            pneumatic2.test();
            logTFEvent(F("EVENT:forearm_pneumatic_test"));
            break;

        case 'y':
            enableForearmPneumatic = true;
            Serial.println(F("[Config] Forearm pneumatic enabled"));
            logTFEvent(F("EVENT:forearm_pneumatic_enabled"));
            break;

        case 'n':
            enableForearmPneumatic = false;
            Serial.println(F("[Config] Forearm pneumatic disabled"));
            logTFEvent(F("EVENT:forearm_pneumatic_disabled"));
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
            enablePalmPneumatic = true;
            Serial.println(F("[Config] Palm pneumatic enabled"));
            logTFEvent(F("EVENT:palm_pneumatic_enabled"));
            break;

        case 'd':
            enablePalmPneumatic = false;
            Serial.println(F("[Config] Palm pneumatic disabled"));
            logTFEvent(F("EVENT:palm_pneumatic_disabled"));
            break;

        case 'h':
            Serial.println(F("===== Command Help ====="));
            Serial.println(F("f -> calibrate flex flat manually"));
            Serial.println(F("b -> calibrate flex bent manually"));
            Serial.println(F("s -> show flex status"));
            Serial.println(F("r -> reset flex calibration"));
            Serial.println(F("i -> print both IMUs once"));
            Serial.println(F("o -> toggle periodic dual-IMU output"));
            Serial.println(F("m -> recapture DTW neutral pose"));
            Serial.println(F("q -> toggle DTW output"));
            Serial.println(F("z -> I2C scan"));
            Serial.println(F("t -> test pneumatic"));
            Serial.println(F("v -> test pressure sensor"));
            Serial.println(F("w -> test vibration motor"));
            Serial.println(F("e -> enable palm pneumatic"));
            Serial.println(F("d -> disable palm pneumatic"));
            Serial.println(F("u -> test forearm pneumatic"));
            Serial.println(F("y -> enable forearm pneumatic"));
            Serial.println(F("n -> disable forearm pneumatic"));
            Serial.println(F("h -> help"));
            break;

        case 'z':
            Serial.println(F("===== I2C Scan ====="));
            Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
            Wire.setClock(100000);
            for (uint8_t addr = 1; addr < 128; addr++) {
                Wire.beginTransmission(addr);
                if (Wire.endTransmission() == 0) {
                    Serial.print(F("Found: 0x"));
                    Serial.println(addr, HEX);
                    delay(50);
                }
            }
            Serial.println(F("Scan done"));
            break;
    }
}

// ============================================================
// 传感器层
// ============================================================
void updateFlex() {
    if (!flex.isCalibrated()) {
        static unsigned long lastFlexPrint = 0;
        if (millis() - lastFlexPrint >= FLEX_PRINT_INTERVAL) {
            lastFlexPrint = millis();
            Serial.print(F("[FLEX] Raw: "));
            Serial.print(flex.getRaw());
            Serial.println(F("  Calibrate: f=flat, b=bent"));
        }
        return;
    }

    if (!flex.isCalibrationValid()) {
        static unsigned long lastFlexPrint = 0;
        if (millis() - lastFlexPrint >= FLEX_PRINT_INTERVAL) {
            lastFlexPrint = millis();
            Serial.print(F("[FLEX] Raw: "));
            Serial.print(flex.getRaw());
            Serial.println(F("  Calibration invalid, recalibrate: f then b"));
        }
        return;
    }

    flex.update();
}

void updateIMUs() {
    imuUpper.update();
    imuLower.update();
    updateDtwFilters();
    combinedPosture = postureClassifier.update(imuUpper, imuLower);

    unsigned long now = millis();
    if (enableDualIMUPrint && now - lastIMUPrintTime >= PRINT_INTERVAL) {
        lastIMUPrintTime = now;
        printDualIMUData();
    }
}

// ============================================================
// 联动层：controlPalmPneumatic()
// 手掌气动：压力传感器按下 → 充气，放开 → 放气
// ============================================================
void controlPalmPneumatic() {
    if (!enablePalmPneumatic) {
        return;
    }

    bool pressurePressed = pressure.isPressed();
    bool currentInflating = pneumatic.getState() == PNEUMATIC_INFLATING ||
                       pneumatic.getState() == PNEUMATIC_HOLDING;

    if (pressurePressed && !currentInflating) {
        // 压力按下，开始充气
        pneumatic.startInflate();
        logTFEvent(F("EVENT:palm_inflate_start"));
    } else if (!pressurePressed && currentInflating) {
        // 压力松开，放气
        pneumatic.startDeflate();
        logTFEvent(F("EVENT:palm_deflate"));

        // 手掌放气时，小臂也一起放气
        if (pneumatic2.isActive()) {
            pneumatic2.startDeflate();
            forearmInflateTriggered = false;
            logTFEvent(F("EVENT:forearm_deflate_palm"));
        }
    }
}

// ============================================================
// 联动层：controlForearmPneumatic()
// 小臂气动：IMU半抬 + Flex弯曲 → 充气 → 压力按下后松开放气
// ============================================================
void controlForearmPneumatic() {
    if (!enableForearmPneumatic) {
        return;
    }

    ArmState imuState = getControlArmState();
    bool pressurePressed = pressure.isPressed();
    bool wasInflating = pneumatic2.isActive();
    bool flexBent = flex.isCalibrated() &&
                    flex.isCalibrationValid() &&
                    flex.getDetailedState() == FLEX_DETAILED_BENT;

    // IMU半抬且Flex稳定弯曲 → 开始充气
    if (imuState == ARM_STATE_HALF_RAISED && flexBent && !wasInflating && !forearmInflateTriggered) {
        pneumatic2.startInflate();
        forearmInflateTriggered = true;
        logTFEvent(F("EVENT:forearm_inflate_start"));
    }

    // 更新充气计时
    if (wasInflating) {
        pneumatic2.update();
    }

    // 气动完成了 → 重置触发标志
    if (forearmInflateTriggered && !pneumatic2.isActive()) {
        forearmInflateTriggered = false;
    }

    // 压力按下后松手 → 放气
    pneumatic2.checkStopOnPressureRelease(lastPressurePressed, !pressurePressed);
    lastPressurePressed = pressurePressed;
}

// ============================================================
// ============================================================
// 联动层：controlVibration()
// D10震动：Half Raised后3秒未到Picking → 脉冲震动（1开2停循环）
// D6震动：Picking后2秒未按压 → 脉冲震动（1开2停循环）
// ============================================================
void controlVibration() {
    if (!enableVibration) {
        return;
    }

    ArmState imuState = getControlArmState();
    bool pressurePressed = pressure.isPressed();

    unsigned long now = millis();

    // IMU状态未就绪时强制停止所有震动
    if (imuState == ARM_STATE_UNKNOWN) {
        vibration.stop();
        vibration2.stop();
        lastPostureState = imuState;
        return;
    }

    // ----- 检测IMU状态变化 -----
    bool postureChanged = (imuState != lastPostureState);
    lastPostureState = imuState;

    // ----- D10震动：HALF_RAISED进入时开始计时 -----
    if (postureChanged && imuState == ARM_STATE_HALF_RAISED) {
        halfRaisedTime = now;
        halfRaisedTriggered = true;
    }
    // 状态离开HALF_RAISED：停止计时
    else if (postureChanged && imuState != ARM_STATE_HALF_RAISED) {
        halfRaisedTriggered = false;
    }

    // 压力按下：停止D10震动
    if (pressurePressed && vibration2.isPulseMode()) {
        vibration2.stop();
    }

    // 触发D10震动：HALF_RAISED状态持续3秒仍未到Picking
    if (halfRaisedTriggered && imuState == ARM_STATE_HALF_RAISED && !pressurePressed) {
        if (now - halfRaisedTime >= 3000 && !vibration2.isPulseMode()) {
            vibration2.startPulse();
        }
    }
    // 到了Picking：停止D10震动
    if (imuState == ARM_STATE_PICKING && vibration2.isPulseMode()) {
        vibration2.stop();
    }

    // ----- D6震动：PICKING进入时开始计时 -----
    if (postureChanged && imuState == ARM_STATE_PICKING) {
        pickingTime = now;
        pickingTriggered = true;
    }
    // 状态离开PICKING：停止计时
    else if (postureChanged && imuState != ARM_STATE_PICKING) {
        pickingTriggered = false;
    }

    // 压力按下：停止D6震动并重置
    if (pressurePressed) {
        vibration.stop();
        pickingTriggered = false;
    }

    // 触发D6震动：PICKING状态持续2秒仍未按压
    if (pickingTriggered && imuState == ARM_STATE_PICKING) {
        if (now - pickingTime >= 2000 && !vibration.isPulseMode()) {
            vibration.startPulse();
        }
    }

    // 更新两个震动模块
    vibration.update();
    vibration2.update();
}

// ============================================================
// 主程序
// ============================================================
void setup() {
    Serial.begin(115200);

    // ESP32 I2C 初始化 (指定 SDA/SCL 引脚)
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(100000);
    delay(100);

    delay(10);
    pneumatic.init(PALM_PUMP_PIN, PALM_VALVE_PIN);
    delay(10);
    pneumatic2.init(FOREARM_PUMP_PIN, FOREARM_VALVE_PIN);
    delay(10);
    vibration.init(MOTOR_PIN);
    delay(10);
    vibration2.init(MOTOR2_PIN);
    delay(10);
    // 再次确保关闭震动（调用类方法）
    vibration.stop();
    vibration2.stop();
    delay(10);
    flex.init();
#if DTW_HAS_FLEX_CALIBRATION
    flex.applyCalibration(DTW_FLEX_STRAIGHT_RAW, DTW_FLEX_BENT_RAW);
#endif
    delay(10);
    pressure.init();
    delay(10);
    // 配置姿态分类器
    postureClassifier.setStableThreshold(3);
    postureClassifier.setSwitchMargin(0.08f);
    delay(10);
    // 初始化双IMU
    bool imu1Ok = imuUpper.begin();
    delay(10);
    bool imu2Ok = imuLower.begin();
    delay(10);
    imuUpper.update();
    imuLower.update();
    updateDtwFilters();
    captureDtwNeutral();
    delay(10);
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

    Serial.println(F("=== System Ready ==="));
    printMemoryInfo();
    Serial.println(F("Two IMUs share I2C bus: IMU1=0x68, IMU2=0x69"));
    Serial.println(F("Palm pneumatic: pressure press -> inflate"));
    Serial.println(F("Forearm pneumatic: IMU half raise -> inflate 9s"));
    Serial.println(F("Use command i to inspect both IMUs, o to toggle streaming."));
    Serial.println(F("DTW: templates loaded from DTWCalibrationData.h, m recaptures neutral."));
    Serial.println(F("h -> View command help"));
}

void loop() {
    handleCommand();              // 命令层
    updateIMUs();                // 传感器层：双IMU更新
    updateFlex();                // 传感器层
    updateDtwMotionCapture();    // DTW：固定长度采样窗口，间隔复核
    pressure.update();            // 传感器层
    controlPalmPneumatic();       // 联动层：手掌气动
    controlForearmPneumatic();    // 联动层：小臂气动
    controlVibration();          // 联动层：震动控制
    vibration2.update();        // 震动计时控制
}
