#include <Arduino.h>
#include <ESP8266WiFi.h> 
#include <DNSServer.h>
#include <AsyncMqttClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#include "config.ino"

AsyncMqttClient mqttClient;

// if doorWorking > 0, the door is currently moving, don't refresh status
int doorWorking = 0; 

// relayTime is another timer (# of 100ms ticks) to leave the relay open
int relayTime = 0; 

int tempTime = TEMP_INTERVAL;

// Seems obvious :)
bool doorOpen = false;

DHT dht(PIN_DHT, DHTTYPE);
char buf[100];

void setup() {

  pinMode(PIN_DOOR_STATUS, INPUT_PULLUP);
  pinMode(PIN_DOOR_CONTROL, OUTPUT);

  dht.begin();

  digitalWrite(PIN_DOOR_CONTROL, false);

  // Initialize Serial Console
  Serial.begin(115200);

  // Start connection to WiFi Network
  Serial.print("Connecting to '");
  Serial.print(WIFI_SSN);
  Serial.println("': ");

  WiFi.begin(WIFI_SSN, WIFI_PW);

  // Wait until we are connected to Wifi
  while(WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  //print out obtained IP address
  Serial.print(" Connected with IP: ");
  Serial.println(WiFi.localIP());

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onMessage(onMqttMessage);

  mqttClient.setClientId(MQTT_CLIENT_ID);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setKeepAlive(5);
  mqttClient.setWill(MQTT_TOPIC_STATUS, 2, true, "offline");
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);

  mqttClient.setSecure(true);
  mqttClient.addServerFingerprint((const uint8_t[])MQTT_SERVER_FINGERPRINT);

  Serial.println("Connecting to MQTT...");
  mqttClient.connect();

}

uint16_t controlSubscribePacketId;

void onMqttConnect(bool sessionPresent) {
  Serial.println("** Connected to the broker **");
  // subscribe to the control topic
  controlSubscribePacketId = mqttClient.subscribe(MQTT_TOPIC_CONTROL_DOOR, 2);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  // once successfully subscribed to control, public online status
  if (packetId == controlSubscribePacketId) {
    Serial.println("** Subscribe acknowledged **");
    mqttClient.publish(MQTT_TOPIC_STATUS, 2, true, "online");
  }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("** Disconnected from the broker **");
  Serial.println("Reconnecting to MQTT...");
  mqttClient.connect();
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("** Publish received **");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  payload: ");
  Serial.println(payload);

  if (strcmp(MQTT_TOPIC_CONTROL_DOOR, topic) == 0)
  {
    if (doorWorking == 0 && !doorOpen && strcmp("open", payload) == 0) {
      doorWorking = DOOR_TRAVEL_TIME;
      relayTime = RELAY_TIME;
      mqttClient.publish(MQTT_TOPIC_STATUS_DOOR, 0, true, "opening");
      Serial.println("Door opening.");
      digitalWrite(PIN_DOOR_CONTROL, true);
    }
    if (doorWorking == 0 && doorOpen && strcmp("close", payload) == 0) {
      doorWorking = DOOR_TRAVEL_TIME;
      relayTime = RELAY_TIME;
      mqttClient.publish(MQTT_TOPIC_STATUS_DOOR, 0, true, "closing");
      Serial.println("Door closing.");
      digitalWrite(PIN_DOOR_CONTROL, true);
    }
  }
}

void loop() {
  if (doorWorking > 0) {
    // if the door is moving, don't refresh door state based on input pin
    doorWorking--;
    if (doorWorking == 0) {
      // door finished moving, push current state
      doorOpen = !digitalRead(PIN_DOOR_STATUS);
      Serial.println(doorOpen ? "Door is now open." : "Door is now closed.");
      mqttClient.publish(MQTT_TOPIC_STATUS_DOOR, 0, true, doorOpen ? "open": "closed");
    }
  } else {
    if (doorOpen != !digitalRead(PIN_DOOR_STATUS)) // door status pin is high when door is closed and low when open
    {
      // door wasn't moving but state changed (i.e. someone pressed the button on the wall)
      doorOpen = !doorOpen;
      Serial.println(doorOpen ? "Door changed to open." : "Door changed to closed.");
      mqttClient.publish(MQTT_TOPIC_STATUS_DOOR, 0, true, doorOpen ? "open": "closed");
    }
  }

  if (relayTime > 0) {
    relayTime--;
    if (relayTime == 0) {
      digitalWrite(PIN_DOOR_CONTROL, false);
    }
  }

  if (tempTime > 0) {
    tempTime--;
    if (tempTime == 0) {
      tempTime = TEMP_INTERVAL;
      float temperature = dht.readTemperature();
      if (!isnan(temperature)) {
        sprintf(buf, "%0.2f", temperature);
        mqttClient.publish(MQTT_TOPIC_STATUS_TEMPERATURE, 0, true, buf);
      }
      float humidity = dht.readHumidity();
      if (!isnan(humidity)) {
        sprintf(buf, "%0.2f", humidity);
        mqttClient.publish(MQTT_TOPIC_STATUS_HUMIDITY, 0, true, buf);
      }
    }
  }

  delay(100);
}
