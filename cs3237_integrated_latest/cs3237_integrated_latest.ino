#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cmath>

// --- ADDED for WiFi & MQTT ---
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
// ---

// PIN
#define DHTPIN 25
#define DHTTYPE DHT11
#define HEART_PIN 32
#define SOUND_PIN 34
#define LDR_PIN 35 // <-- ADDED: LDR Pin. (Note: Pin 16 is not an ADC pin, using 35 instead)

// 变量
#define SAMPLE_SIZE_S 50    // 噪音，50ms 采样窗口
#define NOISE_THRESHOLD 500
#define MIN_CHANGE 7.5
#define ALPHA 0.75
#define HEART_PERIOD_MS 50 // 采样周期 50ms
#define MOTION_THRESHOLD 2.0 // <-- ADDED: Threshold for gyro motion detection (sum of deg/s)

// --- ADDED: WiFi & MQTT ---
// !!! FILL IN YOUR DETAILS HERE !!!
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqttServer = "YOUR_MQTT_BROKER_IP"; // e.g., "192.168.1.100"
const int mqttPort = 1883;
const char* mqttTopic = "esp32/sensor_data"; // Topic to publish to
// ---

// 对象化
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
WiFiClient espClient;
PubSubClient client(espClient);
// ---

// RTOS
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t adcMutex;
SemaphoreHandle_t sensorDataMutex; // <-- ADDED: To protect global data

// --- ADDED: Global Shared Data ---
float g_bpm = 0.0;
float g_temp = 0.0;
float g_hum = 0.0;
int g_noise = 0;
int g_ldr = 0;
int g_in_motion = 0; // <-- ADDED: 0 = not in motion, 1 = in motion

// --- REMOVED: Old MPU struct ---
/*
struct MPUData {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float mpuTemp;
};
MPUData g_mpuData;
*/
// ---

// 函数声明
void Task_HeartRate(void *pvParameters);
void Task_DHT(void *pvParameters);
void Task_Sound(void *pvParameters);
void Task_MPU(void *pvParameters);
void Task_LDR(void *pvParameters);        // <-- ADDED
void Task_UploadData(void *pvParameters); // <-- ADDED
void Task_MQTT_Loop(void *pvParameters);  // <-- ADDED
void setupWiFi();                         // <-- ADDED
void reconnectMQTT();                     // <-- ADDED


void setup() {
  Serial.begin(115200);

  // --- RTOS 对象 ---
  i2cMutex = xSemaphoreCreateMutex();
  adcMutex = xSemaphoreCreateMutex();
  sensorDataMutex = xSemaphoreCreateMutex(); // <-- ADDED

  if (i2cMutex == NULL || adcMutex == NULL || sensorDataMutex == NULL) {
      Serial.println("FATAL: FreeRTOS object creation failed!");
      while (1) delay(1000);
  }
  // ----------------------------------------------------

  pinMode(LDR_PIN, INPUT); // <-- ADDED
  dht.begin();
  Wire.begin();

  // --- MPU 初始化 ---
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    if (mpu.begin()) {
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.println("MPU6050 Initialized.");
    } else {
       Serial.println("Failed to find MPU6050 chip");
    }
    xSemaphoreGive(i2cMutex);
  } else {
    Serial.println("Setup: Failed to get I2C Mutex for init!");
  }

  // --- WiFi & MQTT Setup ---
  setupWiFi();
  client.setServer(mqttServer, mqttPort);
  // ---

  // 创建任务
  // Core 1 (Sensors)
  xTaskCreatePinnedToCore(Task_HeartRate, "HeartRate", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_DHT, "DHT", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Sound, "Sound", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_MPU, "MPU", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Task_LDR, "LDR", 2048, NULL, 1, NULL, 1); // <-- ADDED
  
  // Core 0 (Network)
  xTaskCreatePinnedToCore(Task_UploadData, "Upload", 4096, NULL, 2, NULL, 0); // <-- ADDED
  xTaskCreatePinnedToCore(Task_MQTT_Loop, "MQTTLoop", 4096, NULL, 2, NULL, 0); // <-- ADDED
}


void loop() {
  // FreeRTOS handles the tasks, so loop is empty.
}

// ----------------------------------------------------
// --- ADDED: WiFi & MQTT Helper Functions ---
// ----------------------------------------------------

void setupWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // You can change the client ID if needed
    if (client.connect("esp32Client")) {
      Serial.println("connected");
      // You can also subscribe to topics here if needed
      // client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

// ----------------------------------------------------
// --- 任务定义 (Task Definitions) ---
// ----------------------------------------------------

// 心率计数
void Task_HeartRate(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(HEART_PERIOD_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static float oldValue = 500.0;
    static float thresholdMax = 0.0;
    static unsigned long bpmMills = 0;
    static int bpm_count = 0;
    static unsigned long FirstBpmTime = 0;
    static unsigned long timeBetweenBeats = 0;
    
    if (bpmMills == 0) {
        bpmMills = millis();
        FirstBpmTime = millis();
        timeBetweenBeats = millis();
    }
    int minDelayBetweenBeats = 400;

    while (1) {
        int rawValue; 
        if (xSemaphoreTake(adcMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            rawValue = analogRead(HEART_PIN);
            xSemaphoreGive(adcMutex);
        } else {
            Serial.println("Heart task failed to get ADC mutex");
            rawValue = oldValue; 
        }

        float value = ALPHA * oldValue + (1 - ALPHA) * rawValue;
        float change = value - oldValue;
        oldValue = value;

        if (change < MIN_CHANGE && (millis() - timeBetweenBeats) >= 2000) {
            bpm_count = 0;
        }

        if ((change >= thresholdMax) && (change > MIN_CHANGE) && (change < 30) && (millis() > timeBetweenBeats + minDelayBetweenBeats)) {
            if (bpm_count == 0){
              FirstBpmTime = millis();
            }
            thresholdMax = change;
            timeBetweenBeats = millis();
            bpm_count++;
            Serial.println("Heart beat detected!");
        }
        thresholdMax = thresholdMax * 0.97;

        if (millis() >= bpmMills + 15000) { // Report every 15 seconds
            float calculated_bpm = 0.0;
            if (bpm_count > 0) {
                calculated_bpm = (float)bpm_count * (60000.0 / (timeBetweenBeats - FirstBpmTime)); 
            }
            
            // --- MODIFIED: Write to global variable ---
            if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_bpm = calculated_bpm;
                xSemaphoreGive(sensorDataMutex);
            }
            // ---

            Serial.printf("BPM (approx): %.1f\n", calculated_bpm);
            
            bpm_count = 0;
            bpmMills = millis();
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


// 湿度温度
void Task_DHT(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(2000); // 每2s
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    Serial.printf("Temp: %.1f C, Hum: %.1f %%\n", t, h);
    
    // --- MODIFIED: Write to global variable ---
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (!isnan(t)) g_temp = t; 
      if (!isnan(h)) g_hum = h;
      xSemaphoreGive(sensorDataMutex);
    } 
    // ---
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 噪声监测
void Task_Sound(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // 每100ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    unsigned long startTime = millis();
    int signalMax = 0;
    int signalMin = 1023;

    while (millis() - startTime < SAMPLE_SIZE_S) {
      int sample;
      if (xSemaphoreTake(adcMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          sample = analogRead(SOUND_PIN);
          xSemaphoreGive(adcMutex);
      } else {
          Serial.println("Sound task failed to get ADC mutex");
          sample = 1023; 
      }
      
      if (sample < 1024) {
        if (sample > signalMax) signalMax = sample;
        else if (sample < signalMin) signalMin = sample;
      }
    }

    int amplitude = signalMax - signalMin;
    Serial.printf("Noise amplitude: %d\n", amplitude);

    if (amplitude > NOISE_THRESHOLD) {
      Serial.println("Loud sound detected!");
    }
    
    // --- MODIFIED: Write to global variable ---
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      g_noise = amplitude;
      xSemaphoreGive(sensorDataMutex);
    } 
    // ---

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ----------------------------------------------------
// --- ADDED: New Task for LDR Polling ---
// ----------------------------------------------------
void Task_LDR(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(2000); // Read every 2 seconds
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while(1) {
    int ldrValue;
    // Use ADC mutex to read
    if (xSemaphoreTake(adcMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      ldrValue = analogRead(LDR_PIN);
      xSemaphoreGive(adcMutex);
    } else {
      Serial.println("LDR Task: Failed to get ADC Mutex.");
      ldrValue = -1; // Indicate error
    }

    // Use Sensor Data mutex to write
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      g_ldr = ldrValue;
      xSemaphoreGive(sensorDataMutex);
    } else {
      Serial.println("LDR Task: Failed to get Sensor Mutex.");
    }
    
    Serial.printf("LDR Value: %d\n", ldrValue);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// MPU6050 轮询 (Polling)
void Task_MPU(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Poll every 1 second
  TickType_t xLastWakeTime = xTaskGetTickCount();
  sensors_event_t a, g, temp;

  while (1) {
    // Protect I2C read with mutex
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      mpu.getEvent(&a, &g, &temp);
      xSemaphoreGive(i2cMutex);

      // --- MODIFIED: Calculate motion state ---
      float totalGyro = abs(g.gyro.x) + abs(g.gyro.y) + abs(g.gyro.z);
      int current_motion_state = (totalGyro > MOTION_THRESHOLD) ? 1 : 0;
      
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_in_motion = current_motion_state;
        xSemaphoreGive(sensorDataMutex);
      }
      // ---

      // Print the readings
      Serial.printf("MPU Total Gyro: %.2f, Motion State: %d\n", totalGyro, current_motion_state);
      Serial.println("--------------------");

    } else {
      Serial.println("MPU Task: Failed to get I2C Mutex!");
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ----------------------------------------------------
// --- ADDED: Task for MQTT Connection Management ---
// ----------------------------------------------------
void Task_MQTT_Loop(void *pvParameters) {
  while(1) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        reconnectMQTT();
      }
      client.loop(); // Handle MQTT housekeeping
    } else {
      Serial.println("MQTT Loop: WiFi disconnected.");
    }
    // Check connection status frequently
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ----------------------------------------------------
// --- ADDED: Task for Uploading JSON Data via MQTT ---
// ----------------------------------------------------
void Task_UploadData(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(16000); // 16 seconds
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  // Allocate buffer for JSON payload
  char payload[512];
  // Create JSON document
  StaticJsonDocument<512> doc;

  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (WiFi.status() != WL_CONNECTED || !client.connected()) {
      Serial.println("UploadTask: WiFi or MQTT disconnected. Skipping upload.");
      continue;
    }

    // --- Create JSON Object ---
    // Lock mutex to safely read all global variables
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      doc.clear(); // Clear previous JSON data
      doc["bpm"] = g_bpm;
      doc["temperature"] = g_temp;
      doc["humidity"] = g_hum;
      doc["noise"] = g_noise;
      doc["ldr"] = g_ldr;
      doc["in_motion"] = g_in_motion; 

      // Release the mutex
      xSemaphoreGive(sensorDataMutex);
    } else {
      Serial.println("UploadTask: Failed to get sensor mutex. Skipping.");
      continue;
    }

    // Serialize JSON to string
    serializeJson(doc, payload, sizeof(payload));
    
    Serial.print("Uploading payload: ");
    Serial.println(payload);
    
    // Publish to MQTT topic
    if (!client.publish(mqttTopic, payload)) {
        Serial.println("MQTT Publish failed!");
    }
  }
}