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
const float ANGLE_THRESHOLD = 90.0;           // 到位阈值
const float HYSTERESIS = 5.0;                 // 迟滞范围
const int BENT_MIN_RAW = 270;                  // 弯曲下限（低于此值解除到位）
const unsigned long PRINT_INTERVAL = 500;     // 输出间隔(ms)

// ========= 校准数据 =========
int flatValue = 0;
int bentValue = 0;
bool flatCalibrated = false;
bool bentCalibrated = false;

// ========= 状态 =========
bool bentTriggered = false;
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
  if (bentTriggered) return "到位";
  if (angle <= 20.0) return "伸直";
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
  if (cmd == 'f') {
    flatValue = calibrateAverage(FLEX_PIN, 25);
    flatCalibrated = true;
    smoothRaw = -1;
    bentTriggered = false;
    Serial.println(">> 手臂伸直校准完成");
    Serial.print("Flat = ");
    Serial.println(flatValue);
  }
  else if (cmd == 'b') {
    bentValue = calibrateAverage(FLEX_PIN, 25);
    bentCalibrated = true;
    smoothRaw = -1;
    bentTriggered = false;

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
    bentTriggered = false;
    Serial.println(">> 已重置校准");
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

  if (raw < BENT_MIN_RAW) {
    bentTriggered = false;
  } else if (!bentTriggered && angle >= (ANGLE_THRESHOLD + HYSTERESIS)) {
    bentTriggered = true;
  } else if (bentTriggered && angle <= (ANGLE_THRESHOLD - HYSTERESIS)) {
    bentTriggered = false;
  }

  Serial.print("[FLEX] Raw: ");
  Serial.print(raw);
  Serial.print("  Angle: ");
  Serial.print(angle, 1);
  Serial.print("  Norm: ");
  Serial.print(norm, 3);
  Serial.print("  State: ");
  Serial.print(getStateName(angle));
  if (bentTriggered) Serial.print("  >> 到位!");
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
#define TEST_PPG
// #define TEST_ALL

void loop() {
    // ===== 等待校准完成 =====
    if (!isCalibrated() || !isCalibrationValid()) {
        if (Serial.available()) {
            char cmd = Serial.read();
            if (cmd != '\n' && cmd != '\r') {
                handleFlexCommand(cmd);
            }
        }
        delay(100);
    } else {
        // ===== 校准完成后才运行传感器 =====

        // Flex 传感器命令处理（校准后仍可响应）
        if (Serial.available()) {
            char cmd = Serial.read();
            if (cmd != '\n' && cmd != '\r') {
                handleFlexCommand(cmd);
            }
        }

        // Flex 传感器数据输出
        updateFlexSensor();

#ifdef TEST_PPG
        ppg.update();
        static unsigned long lastPPGPrint = 0;
        if (millis() - lastPPGPrint >= PRINT_INTERVAL) {
            lastPPGPrint = millis();
            ppg.test();
        }
        delay(20);  // 内部采样频率
#endif
    }

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
