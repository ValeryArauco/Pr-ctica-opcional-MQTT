#include <WiFi.h>
#include <PubSubClient.h>

double limit = 100; 
WiFiClient wiFiClient;

// WiFi Configuration
const char *WIFI_SSID = "HONOR Magic5 Lite 5G";
const char *WIFI_PASS = "12345678";

// MQTT Configuration
const char *MQTT_BROKER_HOST = "broker.hivemq.com";
const int MQTT_BROKER_PORT = 1883;
const char *MQTT_CLIENT_ID = "valery.arauco@ucb.edu.bo";
const char *PUBLISH_TOPIC = "ucb/edu/bo/grupo4_publish";//where the maximum range is published from the app
const char *SUBSCRIBE_TOPIC = "ucb/edu/bo/grupo4_receive";//channel from which the ESP receives the distance

// Time Constants
const unsigned long PUBLISH_INTERVAL = 2000;

// Ultrasonic Pin Configuration
const int PIN_TRIGGER = 26;
const int PIN_ECHO = 25;

// LED Pin Configuration
const int LED_PIN = LED_BUILTIN;
const int PIN_RED = 33;
const int PIN_YELLOW = 12;
const int PIN_GREEN = 13;


//Ultrasonic Sensor class
class UltrasonicSensor {
private:
  int triggerPin;
  int echoPin;
public:
  UltrasonicSensor(int triggerPin, int echoPin) : triggerPin(triggerPin), echoPin(echoPin) {}
  double readUltrasonicDistance() {
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    pinMode(echoPin, INPUT);
    return 0.01723 * pulseIn(echoPin, HIGH);
  }
};

//LED class
class LED {
private:
  int pin;
public:
  LED(int pin) : pin(pin) {
    pinMode(pin, OUTPUT);
  }
  void turnOn() {
    digitalWrite(pin, HIGH);
  }

  void turnOff() {
    digitalWrite(pin, LOW);
  }
};

// WiFi Manager Class
class WiFiManager {
private:
  const char *ssid;
  const char *password;
public:
  WiFiManager(const char *ssid, const char *password) : ssid(ssid), password(password) {}

  void connect() {
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(200);
      Serial.print(".");
    }

    Serial.println("Connected!");
  }
};

// MQTT Manager Class
class MQTTManager {
private:
  const char *brokerHost;
  int brokerPort;
  const char *clientId;
  PubSubClient mqttClient;

  static void callback(const char *topic, byte *payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++)
      message += (char)payload[i];

    if (String(topic) == PUBLISH_TOPIC) 
    {
      limit = message.toDouble();
      Serial.print("New limit received: ");
      Serial.println(limit);
    } 
    else if (String(topic) == SUBSCRIBE_TOPIC) {
      Serial.print("Message from topic ");
      Serial.print(topic);
      Serial.print(": ");
      Serial.println(message);
      LED red(PIN_RED), yellow(PIN_YELLOW), green(PIN_GREEN), pin(LED_PIN);

      double distance = message.toDouble();
      red.turnOff();
      yellow.turnOff();
      green.turnOff();
      pin.turnOff(); 
      if (distance >= 2 && distance <= limit / 4) {
        pin.turnOn();
      } else if (distance > limit / 4 && distance <= limit / 4 * 2) {
        red.turnOn();
      } else if (distance > limit / 4 * 2 && distance <= limit / 4 * 3) {
        yellow.turnOn();
      } else if (distance > limit / 4 * 3 && distance <= limit) {
        green.turnOn();
      
      }
    }
  }
public:
  MQTTManager(const char *brokerHost, int brokerPort, const char *clientId)
    : brokerHost(brokerHost), brokerPort(brokerPort), clientId(clientId) {}

  void connect() {
    Serial.print("Connecting to ");
    Serial.println(brokerHost);

    mqttClient.setServer(brokerHost, brokerPort);
    mqttClient.setCallback(callback);

    if (mqttClient.connect(clientId)) {
      Serial.println("Connected!");
      mqttClient.subscribe(SUBSCRIBE_TOPIC);
      mqttClient.subscribe(PUBLISH_TOPIC);
    }
  }

  void loop() {
    mqttClient.loop();
  }

  PubSubClient& getMqttClient() {
    return mqttClient;
  }
  void setClient(WiFiClient& client) {
    mqttClient.setClient(client);
  }
};

//Main
WiFiManager wifiManager(WIFI_SSID, WIFI_PASS);
MQTTManager mqttManager(MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_CLIENT_ID);

unsigned long previousPublishMillis = 0;

void setup() {
  Serial.begin(115200);

  mqttManager.setClient(wiFiClient);
  wifiManager.connect();
  mqttManager.connect();
}

void loop() {
  if (mqttManager.getMqttClient().connected()) {
    unsigned long now = millis();
    if (now - previousPublishMillis >= PUBLISH_INTERVAL) {
      previousPublishMillis = now;
      UltrasonicSensor sensor(PIN_TRIGGER, PIN_ECHO);
      String distance = String(sensor.readUltrasonicDistance(), 2);
      mqttManager.getMqttClient().publish(SUBSCRIBE_TOPIC, distance.c_str());
    }
    mqttManager.getMqttClient().loop();
  } else {
    Serial.println("MQTT broker not connected!");
    delay(2000);
  }
}