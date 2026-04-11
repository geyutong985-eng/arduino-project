// 主循环状态机
// 统一管所有状态，根据 Unity 指令 + 传感器判定切换

#include "ActuatorPneumatic.h"
#include "SensorIMU.h"
#include "SensorPPG.h"
#include "SensorFlex.h"

// ===== 引脚定义 =====
#define PUMP_PIN 8
#define VALVE_PIN 9
const int FLEX_PIN = A3;   // 弯曲传感器引脚
const int PPG_PIN = A0;     // PPG 传感器引脚

// ===== 模块实例 =====
ActuatorPneumatic pneumatic;
SensorIMU imu;
SensorPPG ppg(PPG_PIN, 50);

// ===== 主状态枚举 =====
enum MainState {
    STATE_IDLE,         // 空闲
    STATE_POSE_CHECK,   // 姿态判定
    STATE_ACTUATOR,     // 执行器控制
    STATE_FEEDBACK      // 反馈
};

MainState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

// ========== SensorFlex 内容（手动内联版）==========
// ========= 可配置参数 =========
const int FILTER_SAMPLES = 9;                 // 滤波采样次数
const int MIN_DIFF = 50;                      // 最小校准差值（建议先 50）
const float ANGLE_THRESHOLD = 20.0;           // 到位阈值（伸直=小角度）
const float HYSTERESIS = 5.0;                 // 迟滞范围
const int FLAT_MIN_RAW = 700;                  // 伸直下限（高于此值触发到位）
const unsigned long PRINT_INTERVAL = 500;     // 输出间隔(ms)

// ========= 校准数据 =========
int flatValue = 0;
int bentValue = 0;
bool flatCalibrated = false;
bool bentCalibrated = false;

// ========= 状态 =========
bool flatTriggered = false;
bool isCalibrating = false;  // 校准期间暂停 PPG
float smoothRaw = -1;
unsigned long lastPrintTime = 0;

// ========= 工具函数 =========
void sortArray(int arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - 1 - i; j++) {
      if (arr[j] > arr[j + 1]) {
        int t = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = t;
      }
    }
  }
}

int readMedianRaw(int pin, int samples) {
  if (samples > 25) samples = 25;
  int values[25];
  for (int i = 0; i < samples; i++) {
    values[i] = analogRead(pin);
    delay(2);
  }
  sortArray(values, samples);
  return values[samples / 2];
}

int readStableRaw(int pin) {
  int medianRaw = readMedianRaw(pin, FILTER_SAMPLES);
  if (smoothRaw < 0) {
    smoothRaw = medianRaw;
  } else {
    smoothRaw = 0.75 * smoothRaw + 0.25 * medianRaw;
  }
  return (int)(smoothRaw + 0.5);
}

int calibrateAverage(int pin, int samples = 25) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(4);
  }
  return sum / samples;
}

float getNormalized(int raw, int flatVal, int bentVal) {
  if (flatVal == bentVal) return 0.0;
  float norm = (float)(raw - flatVal) / (float)(bentVal - flatVal);
  return constrain(norm, 0.0, 1.0);
}

float getAngle(int raw, int flatVal, int bentVal) {
  return getNormalized(raw, flatVal, bentVal) * 180.0;
}

bool isCalibrated() {
  return flatCalibrated && bentCalibrated;
}

bool isCalibrationValid() {
  if (!flatCalibrated || !bentCalibrated) return false;
  return abs(flatValue - bentValue) >= MIN_DIFF;
}

const char* getStateName(float angle) {
  if (flatTriggered) return "伸直到位";
  if (angle >= 160.0) return "弯曲";
  return "中间";
}

void printCalibrationStatus() {
  Serial.println("===== 校准状态 =====");
  if (!flatCalibrated) {
    Serial.println("Flat: 未校准");
  } else {
    Serial.print("Flat = ");
    Serial.println(flatValue);
  }
  if (!bentCalibrated) {
    Serial.println("Bent: 未校准");
  } else {
    Serial.print("Bent = ");
    Serial.println(bentValue);
  }
  if (isCalibrated()) {
    Serial.print("Diff = ");
    Serial.println(abs(flatValue - bentValue));
    Serial.print("Valid: ");
    Serial.println(isCalibrationValid() ? "Yes" : "No");
  }
}

// ========== Flex 传感器命令处理 ==========
void handleFlexCommand(char cmd) {
  // 校准时暂停 PPG，避免干扰
  if (cmd == 'f' || cmd == 'b') {
    isCalibrating = true;
  }

  if (cmd == 'f') {
    flatValue = calibrateAverage(FLEX_PIN, 25);
    flatCalibrated = true;
    smoothRaw = -1;
    flatTriggered = false;
    Serial.println(">> 手臂伸直校准完成");
    Serial.print("Flat = ");
    Serial.println(flatValue);
  }
  else if (cmd == 'b') {
    bentValue = calibrateAverage(FLEX_PIN, 25);
    bentCalibrated = true;
    smoothRaw = -1;
    flatTriggered = false;

    Serial.println(">> 手肘弯曲校准完成");
    Serial.print("Flat = ");
    Serial.print(flatValue);
    Serial.print(" | Bent = ");
    Serial.print(bentValue);
    Serial.print(" | Diff = ");
    Serial.print(abs(flatValue - bentValue));
    Serial.print(" | Valid: ");
    Serial.println(isCalibrationValid() ? "Yes" : "No");
  }
  else if (cmd == 's') {
    printCalibrationStatus();
  }
  else if (cmd == 'r') {
    flatValue = bentValue = 0;
    flatCalibrated = bentCalibrated = false;
    smoothRaw = -1;
    flatTriggered = false;
    Serial.println(">> 已重置校准");
  }

  // 校准完成，恢复 PPG
  if (cmd == 'f' || cmd == 'b' || cmd == 'r') {
    isCalibrating = false;
  }
}

// ========== Flex 传感器数据更新 ==========
void updateFlexSensor() {
  unsigned long now = millis();
  if (now - lastPrintTime < PRINT_INTERVAL) return;
  lastPrintTime = now;

  int raw = readStableRaw(FLEX_PIN);

  if (!isCalibrated()) {
    Serial.print("[FLEX] Raw: ");
    Serial.print(raw);
    Serial.println("  (请先校准: f伸直 b弯曲)");
    return;
  }

  if (!isCalibrationValid()) {
    Serial.print("[FLEX] Raw: ");
    Serial.print(raw);
    Serial.println("  (校准差值不足，重新校准)");
    return;
  }

  float norm = getNormalized(raw, flatValue, bentValue);
  float angle = getAngle(raw, flatValue, bentValue);

  if (raw > FLAT_MIN_RAW) {
    flatTriggered = true;
  } else if (!flatTriggered && angle <= (ANGLE_THRESHOLD - HYSTERESIS)) {
    flatTriggered = true;
  } else if (flatTriggered && angle >= (ANGLE_THRESHOLD + HYSTERESIS)) {
    flatTriggered = false;
  }

  Serial.print("[FLEX] Raw: ");
  Serial.print(raw);
  Serial.print("  Angle: ");
  Serial.print(angle, 1);
  Serial.print("  Norm: ");
  Serial.print(norm, 3);
  Serial.print("  State: ");
  Serial.print(getStateName(angle));
  if (flatTriggered) Serial.print("  >> 伸直到位!");
  Serial.println();
}

void setup() {
    Serial.begin(115200);
    pinMode(FLEX_PIN, INPUT);

    pneumatic.init(PUMP_PIN, VALVE_PIN);

    Serial.println("=== Elbow Flex Sensor Sensitive Test ===");
    Serial.println("f -> 校准伸直  b -> 校准弯曲  s -> 状态  r -> 重置");

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
// #define TEST_PPG
// #define TEST_ALL

// TODO: PPG 传感器问题 - 心率测量不准确（显示 160-200），isWear 检测也不准
//       可能需要调整 SensorPPG.cpp 中的心率算法参数或阈值

void loop() {
    // ===== 命令处理（始终响应）=====
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd != '\n' && cmd != '\r') {
            handleFlexCommand(cmd);
        }
    }

    // ===== Flex 传感器更新（始终运行）=====
    updateFlexSensor();

#ifdef TEST_PPG
    // ===== PPG 独立运行，不受 Flex 校准限制 =====
    // 校准期间暂停 PPG，避免干扰 Flex 校准
    if (!isCalibrating) {
        ppg.update();
        static unsigned long lastPPGPrint = 0;
        if (millis() - lastPPGPrint >= PRINT_INTERVAL) {
            lastPPGPrint = millis();
            ppg.test();
        }
    }
    delay(20);  // 内部采样频率
#endif

#ifdef TEST_MODE
    pneumatic.test();
    while (true) { }
#endif

#ifndef TEST_MODE
#ifndef TEST_IMU
#ifndef TEST_PPG
    // ===== 根据弯曲状态控制气动 =====
    // 弯曲时放气，伸直时充气 4 秒（只充一次，需弯曲后才重置）

    static unsigned long inflateStartTime = 0;
    static bool isInflating = false;
    static bool wasFlat = false;      // 记录上次状态
    static bool justInflated = false; // 本次伸直已充气过，需弯曲后才重置

    if (isCalibrated()) {
        if (flatTriggered) {
            // 伸直 → 充气 4 秒（只充一次，需弯曲后才重置）
            if (!isInflating && !justInflated) {
                pneumatic.startInflate();
                inflateStartTime = millis();
                isInflating = true;
                wasFlat = true;
                justInflated = true;  // 标记已充气
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
                justInflated = false;  // 重置，允许下次充气
                Serial.println("[气动] 弯曲 → 放气");
            }
        }
    }
#endif
#endif
#endif
}
