const int flexPin = A0;   // 如果 AO 接 A3，就改成 A3

// ========= 可配置参数 =========
const int FILTER_SAMPLES = 9;                 // 滤波采样次数
const int MIN_DIFF = 50;                      // 最小校准差值（建议先 50）
const float ANGLE_THRESHOLD = 90.0;           // 到位阈值
const float HYSTERESIS = 5.0;                 // 迟滞范围
const int BENT_MIN_RAW = 270;                  // 弯曲下限（低于此值解除到位）
const unsigned long PRINT_INTERVAL = 100;     // 输出间隔(ms)

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

void setup() {
  Serial.begin(115200);
  pinMode(flexPin, INPUT);
  Serial.println("=== Elbow Flex Sensor Sensitive Test ===");
  Serial.println("f -> 校准伸直  b -> 校准弯曲  s -> 状态  r -> 重置");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') return;

    if (cmd == 'f') {
      flatValue = calibrateAverage(flexPin, 25);
      flatCalibrated = true;
      smoothRaw = -1;
      bentTriggered = false;
      Serial.println(">> 手臂伸直校准完成");
      Serial.print("Flat = ");
      Serial.println(flatValue);
    }
    else if (cmd == 'b') {
      bentValue = calibrateAverage(flexPin, 25);
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

  if (!isCalibrated()) return;

  unsigned long now = millis();
  if (now - lastPrintTime < PRINT_INTERVAL) return;
  lastPrintTime = now;

  int raw = readStableRaw(flexPin);
  float norm = getNormalized(raw, flatValue, bentValue);
  float angle = getAngle(raw, flatValue, bentValue);

  if (raw < BENT_MIN_RAW) {
    bentTriggered = false;
  } else if (!bentTriggered && angle >= (ANGLE_THRESHOLD + HYSTERESIS)) {
    bentTriggered = true;
  } else if (bentTriggered && angle <= (ANGLE_THRESHOLD - HYSTERESIS)) {
    bentTriggered = false;
  }

  Serial.print("Raw: ");
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
