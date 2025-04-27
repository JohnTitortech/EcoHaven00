#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>

//======== DEFINES & CONFIGURATION ========
#define RL           10.0f

#define Ro_MQ135     2.4f
#define MQ135_Pin    34
#define m_NH3        -0.417f
#define b_NH3        0.858f
#define m_CO2        -0.19f
#define b_CO2        0.49f
#define m_CO         -0.247f
#define b_CO         0.804
#define m_Alkohol    -0.301f
#define b_Alkohol    0.603

#define Ro_MQ2       2.7
#define MQ2_Pin      35
#define m_LPG2      -0.26f
#define b_LPG2       0.22f
#define m_Metana2   -0.30f
#define b_Metana2    0.29f

#define DHTPIN 23
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

const int relayPins[4] = {19,18,17,16};
const int PIN_AC      = 21;
const int PIN_FAN     = 22;

const char* ssid       = "Farhan";
const char* wifi_pass  = "tanyamama";
const char* mqtt_server= "d58d2856910f44d9a1e8ef55cebf8e63.s1.eu.hivemq.cloud";
const int   mqtt_port  = 8883;
const char* mqtt_user  = "Farhan";
const char* mqtt_passw = "Farhan_gaming20";
const char* sensor_topic = "sensors/data";

WiFiClientSecure espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<200> doc;
  auto err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print("JSON parse error: "); 
    Serial.println(err.c_str());
    return;
  }
  int relayIndex = doc["relay"].as<int>() - 1;
  const char* state = doc["state"];
  Serial.printf("CMD relay %d -> %s\n", relayIndex, state);
  if (relayIndex >= 0 && relayIndex < 4) {
    if (strcmp(state, "ON") == 0) {
      digitalWrite(relayPins[relayIndex], LOW);
    } 
    else if (strcmp(state, "OFF") == 0) {
      digitalWrite(relayPins[relayIndex], HIGH);
    }
  } else {
    Serial.println("Invalid relay index");
  }
}

//======== VARIABEL MOVING AVERAGE UNTUK SEMUA GAS ========
const int numReadings = 5;
float readingsNH3[numReadings];   int idxNH3 = 0;   float totalNH3 = 0.0f;   float amonia = 0.0f;
float readingsCO2[numReadings];   int idxCO2 = 0;   float totalCO2 = 0.0f;   float co2   = 0.0f;
float readingsCO[numReadings];    int idxCO  = 0;   float totalCO  = 0.0f;   float CO   = 0.0f;
float readingsAlk[numReadings];   int idxAlk = 0;   float totalAlk = 0.0f;   float alkohol = 0.0f;
float readingsLPG2[numReadings];  int idxLPG2 = 0; float totalLPG2 = 0.0f; float lpg    = 0.0f;
float readingsMet2[numReadings];  int idxMet2 = 0; float totalMet2 = 0.0f; float metana = 0.0f;
float suhu = 0.0f;

//========= RTOS TASK HANDLE =========
TaskHandle_t TaskSensor;
TaskHandle_t TaskPublish;
TaskHandle_t TaskControl;

//======== FUNGSI =========

void setup() {
  Serial.begin(115200);
  delay(100);

  dht.begin();

  for (int i = 0; i < numReadings; i++) {
    readingsNH3[i]  = 0.0f;
    readingsCO2[i]  = 0.0f;
    readingsCO[i]   = 0.0f;
    readingsAlk[i]  = 0.0f;
    readingsLPG2[i] = 0.0f;
    readingsMet2[i] = 0.0f;
  }
  totalNH3 = totalCO2 = totalCO = totalAlk = totalLPG2 = totalMet2 = 0.0f;

  for (int t = 1; t <= 10; t++) {
    Serial.print("Warming up sensors: "); Serial.println(t);
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Gunakan RTOS delay
  }
  Serial.println("Sensor ready.");

  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, wifi_pass);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Gunakan RTOS delay
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();

  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH);
  }
  pinMode(PIN_AC,  OUTPUT); digitalWrite(PIN_AC,  HIGH);
  pinMode(PIN_FAN, OUTPUT); digitalWrite(PIN_FAN, HIGH);

  // Buat Task
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, &TaskSensor, 1);
  xTaskCreatePinnedToCore(publishTask, "PublishTask", 4096, NULL, 1, &TaskPublish, 1);
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 1, &TaskControl, 1);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}

void sensorTask(void * parameter) {
  for(;;) {
    float ratio135 = readSensorRatio(MQ135_Pin, Ro_MQ135);
    float ratio2   = readSensorRatio(MQ2_Pin,   Ro_MQ2);
    suhu      = dht.readTemperature();

    measureNH3     (ratio135);
    measureCO2     (ratio135);
    measureCO      (ratio135);
    measureAlkohol (ratio135);
    measureLPG2    (ratio2);
    measureMetana2 (ratio2);

    vTaskDelay(2000 / portTICK_PERIOD_MS); // 2 detik
  }
}

void publishTask(void * parameter) {
  for(;;) {
    StaticJsonDocument<256> doc;
    doc["lpg"]     = lpg;
    doc["metana"]  = metana;
    doc["CO"]      = CO;
    doc["amonia"]  = amonia;
    doc["alkohol"] = alkohol;
    doc["co2"]     = co2;
    doc["suhu"]    = suhu;

    char buffer[256];
    serializeJson(doc, buffer);
    client.publish(sensor_topic, buffer);
    Serial.print("Published: "); Serial.println(buffer);

    vTaskDelay(5000 / portTICK_PERIOD_MS); // 5 detik
  }
}

void controlTask(void * parameter) {
  for(;;) {
    if (suhu >= 35) {
      digitalWrite(PIN_AC, LOW);
    } else {
      digitalWrite(PIN_AC, HIGH);
    }

    if ( lpg >= 1 || metana >= 1 || CO >= 0.1 || amonia >= 0.01 || alkohol >= 0.01 || co2 >= 0.001) {
      digitalWrite(PIN_FAN, LOW);
    } else {
      digitalWrite(PIN_FAN, HIGH);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 detik
  }
}

void reconnect() {
  while (!client.connected()) {
    String id = "ESP32Client-" + String(random(0xffff), HEX);
    Serial.print("Attempting MQTT connect...");
    if (client.connect(id.c_str(), mqtt_user, mqtt_passw)) {
      Serial.println(" connected");
      client.subscribe("esp32/relay/+");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 2s");
      delay(2000);
    }
  }
}

float readSensorRatio(int pin, float Ro) {
  float raw = analogRead(pin);
  float VRL = raw * (3.3f / 4095.0f);
  float RS  = (3.3f / VRL - 1.0f) * RL;
  return RS / Ro;
}

void measureNH3(float ratio) {
  float ppm = powf(10.0f, (log10f(ratio) - b_NH3) / m_NH3);
  totalNH3             -= readingsNH3[idxNH3];
  readingsNH3[idxNH3]   = ppm;
  totalNH3             += ppm;
  idxNH3               = (idxNH3 + 1) % numReadings;
  amonia               = (totalNH3 / numReadings)*10;
  Serial.print("NH3: "); Serial.println(amonia, 2);
}

void measureCO2(float ratio) {
  float ppm = powf(10.0f, (log10f(ratio) - b_CO2) / m_CO2);
  totalCO2             -= readingsCO2[idxCO2];
  readingsCO2[idxCO2]   = ppm;
  totalCO2             += ppm;
  idxCO2               = (idxCO2 + 1) % numReadings;
  co2                  = (totalCO2 / numReadings)*1000000;
  Serial.print("CO2: "); Serial.println(co2, 2);
}

void measureCO(float ratio) {
  float ppm = powf(10.0f, (log10f(ratio) - b_CO) / m_CO);
  totalCO              -= readingsCO[idxCO];
  readingsCO[idxCO]     = ppm;
  totalCO              += ppm;
  idxCO                = (idxCO + 1) % numReadings;
  CO                  = (totalCO / numReadings)*10000;
  Serial.print("CO: "); Serial.println(CO, 2);
}

void measureAlkohol(float ratio) {
  float ppm = powf(10.0f, (log10f(ratio) - b_Alkohol) / m_Alkohol);
  totalAlk             -= readingsAlk[idxAlk];
  readingsAlk[idxAlk]   = ppm;
  totalAlk             += ppm;
  idxAlk               = (idxAlk + 1) % numReadings;
  alkohol              = (totalAlk / numReadings)*1000;
  Serial.print("Alkohol: "); Serial.println(alkohol, 2);
}

void measureLPG2(float ratio) {
  float ppm = powf(10.0f, (log10f(ratio) - b_LPG2) / m_LPG2);
  totalLPG2            -= readingsLPG2[idxLPG2];
  readingsLPG2[idxLPG2] = ppm;
  totalLPG2            += ppm;
  idxLPG2              = (idxLPG2 + 1) % numReadings;
  lpg                  = (totalLPG2 / numReadings)*10000;
  Serial.print("LPG: "); Serial.println(lpg, 2);
}

void measureMetana2(float ratio) {
  float ppm = powf(10.0f, (log10f(ratio) - b_Metana2) / m_Metana2);
  totalMet2            -= readingsMet2[idxMet2];
  readingsMet2[idxMet2] = ppm;
  totalMet2            += ppm;
  idxMet2              = (idxMet2 + 1) % numReadings;
  metana               = (totalMet2 / numReadings)*1000;
  Serial.print("Metana: "); Serial.println(metana, 2);
}