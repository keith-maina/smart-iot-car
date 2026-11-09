// Smart IoT Car Simulation (ESP32 + Wokwi)
// Ultrasonic distance sensing -> decides GO / SLOW / STOP
// WiFi + MQTT: publishes live distance, accepts remote commands

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// ---------- Pins ----------
#define TRIG_PIN   5
#define ECHO_PIN   18
#define LED_GREEN  25   // GO
#define LED_YELLOW 26   // SLOW
#define LED_RED    27   // STOP
#define BUZZER_PIN 14
#define SERVO_PIN  13   // steering
#define PIR_PIN    19   // person detector

// ---------- WiFi / MQTT ----------
const char* WIFI_SSID = "Wokwi-GUEST";   // Wokwi's built-in WiFi
const char* WIFI_PASS = "";
const char* MQTT_HOST = "broker.hivemq.com";
const int   MQTT_PORT = 1883;
// Change "keith" to something unique so others don't clash with your topics
const char* TOPIC_DIST = "keith/car/distance";
const char* TOPIC_STATE = "keith/car/state";
const char* TOPIC_PERSON = "keith/car/person";
const char* TOPIC_CMD  = "keith/car/cmd";   // send "stop" or "go"

// ---------- Thresholds (cm) ----------
const int STOP_CM = 20;
const int SLOW_CM = 50;

WiFiClient espClient;
PubSubClient mqtt(espClient);
Servo steering;

bool remoteStop = false;
unsigned long lastPublish = 0;


// Buzzer helpers: work on both Arduino-ESP32 core 2.x and 3.x
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  void buzzOn()  { tone(BUZZER_PIN, 1000); }
  void buzzOff() { noTone(BUZZER_PIN); }
#else
  void buzzOn()  { ledcAttachPin(BUZZER_PIN, 10); ledcWriteTone(10, 1000); }
  void buzzOff() { ledcWriteTone(10, 0); }
#endif

// ---------- Ultrasonic ----------
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout
  if (duration == 0) return 400;                  // nothing detected
  return duration * 0.0343 / 2.0;
}

void setState(const char* state);

// ---------- Outputs ----------
void setState(const char* state) {
  digitalWrite(LED_GREEN,  strcmp(state, "GO") == 0);
  digitalWrite(LED_YELLOW, strcmp(state, "SLOW") == 0);
  digitalWrite(LED_RED,    strcmp(state, "STOP") == 0);

  if (strcmp(state, "STOP") == 0) {
    buzzOn();
    steering.write(135);        // steer away
  } else if (strcmp(state, "SLOW") == 0) {
    buzzOff();
    steering.write(110);
  } else {
    buzzOff();
    steering.write(90);         // straight
  }
}

// ---------- MQTT ----------
void onMessage(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.toLowerCase();
  Serial.println("Command received: " + msg);
  if (msg == "stop") remoteStop = true;
  if (msg == "go")   remoteStop = false;
}

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println(" connected!");
}

unsigned long lastMqttTry = 0;
void tryMQTT() {
  if (millis() - lastMqttTry < 5000) return;   // try every 5 s, never block
  lastMqttTry = millis();
  String id = "smartcar-" + String(random(0xffff), HEX);
  if (mqtt.connect(id.c_str())) {
    Serial.println("MQTT connected");
    mqtt.subscribe(TOPIC_CMD);
  } else {
    Serial.println("MQTT not connected yet (car still works offline)");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  steering.attach(SERVO_PIN);
  steering.write(90);

  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (mqtt.connected()) mqtt.loop();
    else tryMQTT();
  }

  float d = readDistanceCm();

  // If a person is detected, the car becomes more cautious
  bool person = digitalRead(PIR_PIN) == HIGH;
  int stopLimit = person ? 40 : STOP_CM;
  int slowLimit = person ? 80 : SLOW_CM;

  const char* state;
  if (remoteStop || d < stopLimit) state = "STOP";
  else if (d < slowLimit)          state = "SLOW";
  else                             state = "GO";
  setState(state);

  // Publish twice a second
  if (millis() - lastPublish > 500) {
    lastPublish = millis();
    char buf[16];
    dtostrf(d, 1, 1, buf);
    if (mqtt.connected()) {
      mqtt.publish(TOPIC_DIST, buf);
      mqtt.publish(TOPIC_STATE, state);
      mqtt.publish(TOPIC_PERSON, person ? "1" : "0");
    }
    Serial.printf("Distance: %s cm | Person: %d | State: %s\n", buf, person, state);
  }
  delay(50);
}
