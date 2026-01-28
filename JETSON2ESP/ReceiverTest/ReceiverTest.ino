const int LED_PIN = 2;

unsigned long blinkPeriod = 1000;
unsigned long lastToggleTime = 0;
bool ledState = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int newPeriod = input.toInt();

    if (newPeriod > 0) {
      blinkPeriod = newPeriod;
    }
  }

  unsigned long now = millis();
  if (now - lastToggleTime >= blinkPeriod) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    lastToggleTime = now;
  }
}
