#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include "arduino_secrets.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#include "tds.h"

#define TEMP_SENSOR_PIN 2
#define TdsSensorPin A1
#define PH_SENSOR_PIN A2

char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = SECRET_BROKER;
int port = 1883;
const char topic[] = "measurement/";

const long interval = 30 * 1000;
unsigned long previousMillis = 0;

unsigned long int avgValue;
float b;
int buf[10],temp;

OneWire ds(TEMP_SENSOR_PIN);
DallasTemperature temp_sensor(&ds);

void setup() {

  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  temp_sensor.begin();
  pinMode(TdsSensorPin,INPUT);
  connectWifi(ssid,pass);
  connectMQTT(broker,port);
  
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
    String measurementsJson = createMeasurementsJson(temp, ph, tds, false, 2);

    Serial.print("Sending measurements: ");
    Serial.println(measurementsJson);

    mqttClient.beginMessage(topic);
    mqttClient.print(measurementsJson);
    mqttClient.endMessage();
  }
}

void connectWifi(const char* ssid, const char* pass) {
  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();
}

void connectMQTT(const char* broker, int port){
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1)
      ;
  }
  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

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

float getTemp() {
  temp_sensor.requestTemperatures();
  Serial.print("Temp: "); 
  Serial.print( temp_sensor.getTempCByIndex(0)); 
  Serial.println("C"); 
  return temp_sensor.getTempCByIndex(0);
  delay(1000);
}

float getPh() {
   for(int i=0;i<10;i++) //Get 10 sample value from the sensor for smooth the value
      { 
        buf[i]=analogRead(PH_SENSOR_PIN);
        delay(10);
      }
    for(int i=0;i<9;i++) //sort the analog from small to large
      {
         for(int j=i+1;j<10;j++)
           {
             if(buf[i]>buf[j])
               {
                 temp=buf[i];
                 buf[i]=buf[j];
                 buf[j]=temp;
               }
           }
      }
    avgValue=0;
    for(int i=2;i<8;i++) //take the average value of 6 center sample
    avgValue+=buf[i];
    float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
    phValue=2.13*phValue; //convert the millivolt into pH value
    Serial.print("Ph: "); 
    Serial.print(phValue,2);
    Serial.println(" ");
    return phValue;
}