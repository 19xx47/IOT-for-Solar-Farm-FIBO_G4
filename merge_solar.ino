
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Wire.h"
#include "Adafruit_INA219.h"
// Data wire is plugged TO GPIO 4
#define ONE_WIRE_BUS 25

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

String payload;
#define Addr 0x4A
Adafruit_INA219 ina219_a;
Adafruit_INA219 ina219_b(0x40);
int sensor = 4;
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "12345678"
#define WIFI_PASSWORD "1212312121"

// Digital Ocean MQTT Mosquitto Broker
#define MQTT_HOST IPAddress(10,61,200,42)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1890

#define MQTT_USERNAME "tonton"
#define MQTT_PASSWORD "fiboxx"

// Test MQTT Topic
#define MQTT_PUB_TEST "G4"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 1000;         // Interval at which to publish sensor readings

int i = 0;


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup(void) {
  
  Serial.begin(115200);
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);

  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();
 Serial.begin(115200);
   ina219_a.begin();
   ina219_b.begin();
   sensors.begin();
  Wire.begin(21, 22);
  Wire.beginTransmission(Addr);
  Wire.write(0x02);
  Wire.write(0x40);
  Wire.endTransmission();
  Serial.print("BV");
  Serial.print("\t");  // Bus Voltage
  Serial.print("SV");
  Serial.print("\t");  // Shunt Voltage
  Serial.print("LV");
  Serial.print("\t");  // Load Voltage
  Serial.print("C");
  Serial.print("\t");  // Current
  Serial.print("P");
  Serial.print("\t");  // Power
  Serial.print("BV");
  Serial.print("\t");  // Bus Voltage
  Serial.print("SV");
  Serial.print("\t");  // Shunt Voltage
  Serial.print("LV");
  Serial.print("\t");  // Load Voltage
  Serial.print("C");
  Serial.print("\t");   // Current
  Serial.println("P");  // Power

 
}


 
void loop(void) {
   float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219_a.getShuntVoltage_mV();
  busvoltage = ina219_a.getBusVoltage_V();
  current_mA = ina219_a.getCurrent_mA();
  power_mW = ina219_a.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  Serial.print(busvoltage);
  Serial.print("\t");
  Serial.print(shuntvoltage);
  Serial.print("\t");
  Serial.print(loadvoltage);
  Serial.print("\t");
  Serial.print(current_mA);
  Serial.print("\t");
  Serial.print(power_mW);
  Serial.print("\t");

  shuntvoltage = ina219_b.getShuntVoltage_mV();
  busvoltage = ina219_b.getBusVoltage_V();
  current_mA = ina219_b.getCurrent_mA();
  power_mW = ina219_b.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  Serial.print(busvoltage);
  Serial.print("\t");
  Serial.print(shuntvoltage);
  Serial.print("\t");
  Serial.print(loadvoltage);
  Serial.print("\t");
  Serial.print(current_mA);
  Serial.print("\t");
  Serial.println(power_mW);
  unsigned int dataone[2];
  Wire.beginTransmission(Addr);
  Wire.write(0x03);
  Wire.endTransmission();

  // Request 2 bytes of data
  Wire.requestFrom(Addr, 2);

  // Read 2 bytes of data luminance msb, luminance lsb
  if (Wire.available() == 2) {
    dataone[0] = Wire.read();
    dataone[1] = Wire.read();
  }

  // Convert the data to lux
  int exponent = (dataone[0] & 0xF0) >> 4;
  int mantissa = ((dataone[0] & 0x0F) << 4) | (dataone[1] & 0x0F);
  float luminance = pow(2, exponent) * mantissa * 0.045;

  Serial.print("Ambient Light luminance 1 :");
  Serial.print(luminance);
  Serial.println(" lux");
float lux = analogRead(sensor) * 0.64453125;
Serial.println(lux);

delay(100);
Serial.println("Requesting temperatures...");
sensors.requestTemperatures(); //อ่านข้อมูลจาก library
Serial.print("Temperature is: ");
Serial.print(sensors.getTempCByIndex(0)); // แสดงค่า อูณหภูมิ 
Serial.println(" *1-C");
Serial.print(sensors.getTempCByIndex(1)); // แสดงค่า อูณหภูมิ 
Serial.println(" *2-C");
delay(1000);


  payload += busvoltage;
  payload += ",";

  payload += shuntvoltage;
  payload += ",";

  payload += loadvoltage;
  payload += ",";

  payload += current_mA;
  payload += ",";

  payload += power_mW;
  payload += ",";

    payload += busvoltage;
  payload += ",";

  payload += shuntvoltage;
  payload += ",";

  payload += loadvoltage;
  payload += ",";

  payload += current_mA;
  payload += ",";

  payload += power_mW;
  payload += ",";


  payload += luminance;
  payload += ",";

  payload += sensors.getTempCByIndex(1);
  payload += ",";

  payload += sensors.getTempCByIndex(0);
  payload += ",";
  

  int payload_len = payload.length() + 1;
  char char_array[payload_len];
  payload.toCharArray(char_array, payload_len);
  Serial.println(char_array);
  mqttClient.publish(MQTT_PUB_TEST, 1, true, char_array);   
  payload = "";
  delay(500);
}

