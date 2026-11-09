// Smart IoT Car Simulation (ESP32 + Wokwi)
// Ultrasonic distance sensing -> decides GO / SLOW / STOP
// WiFi + MQTT: publishes live distance, accepts remote commands

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
const char* TOPIC_SETDIST = "keith/car/setdist";     // dashboard sets distance
const char* TOPIC_SETPERSON = "keith/car/setperson"; // dashboard sets person 1/0
const char* TOPIC_MODE = "keith/car/mode";           // "sim" or "sensor"
const char* TOPIC_CONFIG = "keith/car/config";       // "stopCm,slowCm"
const char* TOPIC_CMD  = "keith/car/cmd";   // send "stop" or "go"

// ---------- Thresholds (cm) ----------
int STOP_CM = 20;
int SLOW_CM = 50;

WiFiClient espClient;
PubSubClient mqtt(espClient);
Servo steering;

bool remoteStop = false;
bool useSim = false;        // true = use distance sent from dashboard
bool simPerson = false;
float simDist = 100;
unsigned long lastPublish = 0;

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

// ---------- Outputs ----------
void setState(const char* state) {
  digitalWrite(LED_GREEN,  strcmp(state, "GO") == 0);
  digitalWrite(LED_YELLOW, strcmp(state, "SLOW") == 0);
  digitalWrite(LED_RED,    strcmp(state, "STOP") == 0);

  if (strcmp(state, "STOP") == 0) {
    tone(BUZZER_PIN, 1000);
    steering.write(135);        // steer away
  } else if (strcmp(state, "SLOW") == 0) {
    noTone(BUZZER_PIN);
    steering.write(110);
  } else {
    noTone(BUZZER_PIN);
    steering.write(90);         // straight
  }
}

// ---------- MQTT ----------
void onMessage(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.toLowerCase();
  Serial.println("Command received: " + msg);
  String t = String(topic);
  if (t == TOPIC_CMD) {
    if (msg == "stop") remoteStop = true;
    if (msg == "go")   remoteStop = false;
  } else if (t == TOPIC_SETDIST) {
    simDist = msg.toFloat();
  } else if (t == TOPIC_SETPERSON) {
    simPerson = (msg == "1");
  } else if (t == TOPIC_MODE) {
    useSim = (msg == "sim");
  } else if (t == TOPIC_CONFIG) {
    int c = msg.indexOf(',');
    if (c > 0) {
      STOP_CM = msg.substring(0, c).toInt();
      SLOW_CM = msg.substring(c + 1).toInt();
    }
  }
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
    mqtt.subscribe(TOPIC_SETDIST);
    mqtt.subscribe(TOPIC_SETPERSON);
    mqtt.subscribe(TOPIC_MODE);
    mqtt.subscribe(TOPIC_CONFIG);
  } else {
    Serial.printf("MQTT failed, error code %d (car still works offline)\n", mqtt.state());
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

  float d = useSim ? simDist : readDistanceCm();

  // If a person is detected, the car becomes more cautious
  bool person = useSim ? simPerson : (digitalRead(PIR_PIN) == HIGH);
  int stopLimit = person ? STOP_CM * 2 : STOP_CM;
  int slowLimit = person ? SLOW_CM * 2 : SLOW_CM;

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
    Serial.printf("WiFi: %s | MQTT: %s\n", WiFi.status() == WL_CONNECTED ? "connected" : "NOT connected", mqtt.connected() ? "connected" : "NOT connected");
    Serial.printf("Distance: %s cm | Person: %d | State: %s\n", buf, person, state);
  }
  delay(50);
}
