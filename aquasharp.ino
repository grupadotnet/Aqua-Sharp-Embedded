#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include "arduino_secrets.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#include "tds.h"

#define TEMP_SENSOR_PIN 2
#define PH_SENSOR_PIN A1
#define TDS_SENSOR_PIN A2

const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;
WiFiSSLClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = SECRET_BROKER;
int port = 8883;
char device_id[] = SECRET_DEVICEID;
String topic = "devices/" + (String)device_id + "/messages/events/";

const char username[] = SECRET_USERNAME;
const char password[] = SECRET_PASSWORD;

const long interval = 30 * 1000;
unsigned long previousMillis = 0;

unsigned long int avgValue;
int buf[50];
int PHSamples = 15; //number of ph samples to be sampled

OneWire ds(TEMP_SENSOR_PIN);
DallasTemperature temp_sensor(&ds);

void setup() {
  Serial.begin(9600);
  if (!Serial.available()) {
    printf("Couldn't connect to serial");
  }
  
  initSensors();
  connectWifi(ssid,pass);
  connectMQTT(broker, port, username, password, device_id);
  Serial.println(topic);

  pinMode(LED_BUILTIN, OUTPUT);
  for(int i=0; i<10; i++) {
    pinMode(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void loop() {
  mqttClient.poll();

  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float temp = getTemp();
    float ph = getPh();
    uint tds = getTds();
    String measurementsJson = createMeasurementsJson(temp, ph, tds, false, 4);

    Serial.print("Sending measurements: ");
    Serial.println(measurementsJson);

    mqttClient.beginMessage(topic);
    mqttClient.print(measurementsJson);
    mqttClient.endMessage();
  }
}

void initSensors(void) {
  temp_sensor.begin();
  pinMode(TdsSensorPin,INPUT);
}

void connectWifi(const char* ssid, const char* pass) {
  status = WiFi.begin(ssid, pass);
  Serial.print("Connecting to Wifi: ");
  Serial.println(ssid);

  uint8_t retry_count = 0;
  while(status != WL_CONNECTED) {
    if(retry_count>=3) {
      Serial.print("Couldn't connect to the wifi network. Status: ");
      Serial.println(status);
      while(1);
    }
    Serial.println("Retrying...");
    retry_count++;
    delay(5000);
  }
  Serial.print("Connected to wifi. IP: ");
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
  Serial.println();
}

void connectMQTT(const char* broker, int port, const char* username, const char* password, const char* device_id) {
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(broker);

  mqttClient.setId(device_id);
  mqttClient.setUsernamePassword(username, password);

  uint8_t retry_count = 0;
  while (!mqttClient.connect(broker, port)) {
    if(retry_count>=3) {
      Serial.print("Couldn't connect to mqtt broker. Error:");
      Serial.println(mqttClient.connectError());
      while(1);
    }
    Serial.println("Retrying...");
    retry_count++;
    delay(5000);
  }
  Serial.println();
  Serial.println("You're connected to the MQTT broker");
}

String createMeasurementsJson(float temp, float ph, uint tds, bool lightOn, int aquariumId) {
  StaticJsonDocument<200> doc;
  doc["Temperature"] = temp;
  doc["Ph"] = ph;
  doc["TDS"] = tds;
  doc["LightOn"] = lightOn;
  doc["AquariumId"] = aquariumId;
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

float getTemp(void) {
  temp_sensor.requestTemperatures(); 
  return temp_sensor.getTempCByIndex(0);
}

float getPh() {
  // Get PHSamples sample values from the sensor for smoothing
  for (int i = 0; i < PHSamples; i++){      
    buf[i] = analogRead(PH_SENSOR_PIN);
    delay(10);
  }
  avgValue = 0;
  // Take the average value of the central 15 samples
  for (int i = 5; i <PHSamples; i++){
    avgValue += buf[i];
  }
  // Convert the analog into millivolt
  float phValue = (float)avgValue * 5.0 / 1024 / PHSamples;
  // Convert the millivolt into pH value
  phValue = phValue*4.49; 
  return phValue;
}
