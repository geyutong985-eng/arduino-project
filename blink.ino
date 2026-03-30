/*
  Blink - LED闪烁示例
  打开Arduino IDE上传到开发板即可运行
*/

// 定义LED引脚（板载LED通常在13号引脚）
#define LED_PIN 13

void setup() {
  // 初始化引脚为输出模式
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);  // 点亮LED
  delay(1000);                  // 等待1秒
  digitalWrite(LED_PIN, LOW);   // 熄灭LED
  delay(1000);                  // 等待1秒
}