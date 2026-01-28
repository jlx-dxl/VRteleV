const int LED_PIN = 2;   // ESP32-WROOM-32 板载 LED

unsigned long blinkPeriod = 0;      // 闪烁周期，0 表示不闪
unsigned long blinkDuration = 0;    // 总闪烁时长
unsigned long blinkStartTime = 0;   // 本次闪烁开始时间
unsigned long lastToggleTime = 0;

bool ledState = false;
bool blinkingActive = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);   // 默认熄灭
  Serial.begin(115200);
}

void loop() {
  // 1. 接收来自发送端的指令
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    int spaceIndex = line.indexOf(' ');   // 协议: 用空格分隔周期和持续时间
    if (spaceIndex > 0) {
      unsigned long period = line.substring(0, spaceIndex).toInt() / 2;
      unsigned long duration = line.substring(spaceIndex + 1).toInt();

      if (period > 0 && duration > 0) {
        blinkPeriod = period;
        blinkDuration = duration;
        blinkStartTime = millis();
        lastToggleTime = millis();
        blinkingActive = true;
        ledState = false;
        digitalWrite(LED_PIN, LOW);
      }
    }
  }

  // 2. 如果当前处于闪烁状态，处理 LED 逻辑
  if (blinkingActive) {
    unsigned long now = millis();

    // 到时间自动停止闪烁
    if (now - blinkStartTime >= blinkDuration) {
      blinkingActive = false;
      digitalWrite(LED_PIN, LOW);
      return;
    }

    // 正常按周期翻转 LED
    if (now - lastToggleTime >= blinkPeriod) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lastToggleTime = now;
    }
  }
}
