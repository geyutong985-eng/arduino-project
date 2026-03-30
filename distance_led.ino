const int trigPin = 9;    // 超声波 Trig 接 D9
const int echoPin = 10;   // 超声波 Echo 接 D10
const int ledPin  = 11;   // LED 接 D11（要用支持 PWM 的引脚）

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600); // 打开串口监视器，方便看距离
}

void loop() {
  // 1. 发送超声波脉冲
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 2. 读取回波时间，并换算成距离（单位：cm）
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration / 58;

  // 3. 限制距离范围，避免数据异常
  distance = constrain(distance, 2, 50);

  // 4. 将距离映射成亮度
  // 距离 2cm -> 亮度 255
  // 距离 50cm -> 亮度 0
  int brightness = map(distance, 2, 50, 255, 0);

  // 5. 输出 PWM 控制 LED 亮度
  analogWrite(ledPin, brightness);

  // 6. 串口输出，方便观察
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm   ");
  Serial.print("Brightness: ");
  Serial.println(brightness);

  delay(50);
}
