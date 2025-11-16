#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cmath>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "ESP32MQTTClient.h"
#include "esp_idf_version.h"
#include "esp_event.h"
#include <PubSubClient.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// PIN
#define LDR_PIN 33
#define DHTPIN 25
#define DHTTYPE DHT11
#define HEART_PIN 32
#define SOUND_PIN 34
// #define buzzerPin 8 // 警报信号
#define MPU_INT_PIN 4   // MPU6050 INT 引脚
#define HEART_LED_PIN 2 // 新增: 脉搏指示灯（假设为 ESP32 内置 LED）
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 48

// --- ADDED: RGB LED PINS ---
#define LED_RED 12
#define LED_GREEN 13
#define LED_BLUE 14

// 变量
#define SAMPLE_SIZE_H 4  // 心率，4次采样
#define SAMPLE_SIZE_S 50 // 噪音，50ms 采样窗口
#define NOISE_THRESHOLD 500
#define MIN_CHANGE 7.5
#define ALPHA 0.75
#define HEART_PERIOD_MS 50        // 采样周期 50ms
#define MQTT_KEEP_ALIVE_SEC 60    // Keep Alive 时间
#define PUBLISH_INTERVAL_MS 30000 // 发布间隔c

// --- WiFi & HTTP 常量 ---
const char *ssid = "CPLADRAGON";
const char *password = "10293847";
// const char* ssid = "05400";
// const char* password = "180098123";
const char *device_name = "Boyu";

// --- MQTT 常量 ---
const char *mqtt_server = "178.128.94.113";
const char *mqtt_topic = "esp32/sensor_data";
const char *mqtt_command_topic = "esp32/prediction"; // Topic to SEND on

// MPU6050 寄存器地址和配置
#define MPU6050_ADDRESS 0x68
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A    // 中断状态寄存器地址
#define MPU_INT_CONFIG 0x20        // 0x37 寄存器值：高电平有效，推挽输出，锁存直到读取
#define MPU_INT_ENABLE_MOT_EN 0x40 // 0x38 寄存器值：只启用运动检测中断
#define MPU6050_MOT_THR 0x1F       // 运动检测阈值寄存器
#define MPU6050_MOT_DUR 0x20       // 运动检测持续时间寄存器

// 对象化
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
// ESP32MQTTClient mqttClient;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
WiFiClient espClient;
PubSubClient client(espClient);
SemaphoreHandle_t fallDetectedSemaphore;
SemaphoreHandle_t motionDetectedSemaphore;
SemaphoreHandle_t i2cMutex; // I2C 互斥锁
SemaphoreHandle_t adcMutex;
SemaphoreHandle_t sensorDataMutex; // OLED 互斥锁

// 共享传感器数据
float g_bpm = 0.0;
float g_temp = 0.0;
float g_hum = 0.0;
int g_noise_peak = 0;
double g_noise_sum = 0.0;     // 噪声总和
int g_noise_sample_count = 0; // 噪声采样次数
volatile bool g_mqttConnected = false;
float g_ldr = 0.0;
int g_in_motion = 1;


// RGB LED State Enum 
enum LedState
{
  STATE_NORMAL,
  STATE_HAPPY,
  STATE_STRESSED
};
volatile LedState g_currentState = STATE_NORMAL; // Default to normal.
volatile float g_happiness_score = 0.0;
// For "Happy" state (Breathing)
float happyPulseSpeed = 0.005; // Controls the "breathing" speed

// For "Stressed" state (Blinking)
unsigned long stressedLastUpdate = 0;
int stressedBlinkSpeed = 250; // 250ms on, 250ms off
bool stressedState = false;   // false = off, true = on

// 函数声明
void Task_HeartRate(void *pvParameters);
void Task_DHT(void *pvParameters);
void Task_Sound(void *pvParameters);
void Task_Motion(void *pvParameters);
void Task_Alarm(void *pvParameters);
void IRAM_ATTR MPU_ISR();
void mpu_write_register(uint8_t reg_addr, uint8_t data);
void Task_LDR(void *pvParameters);
void Task_UploadData(void *pvParameters);
void Task_MQTT_Loop(void *pvParameters);
void reconnectMQTT();
void mqttCallback(char *topic, byte *payload, unsigned int length); // <-- ADDED: MQTT Callback
void Task_LED_Control(void *pvParameters);                          // <-- ADDED: LED Task
void Task_OLED(void *pvParameters);

// --- ADDED: LED Helper Functions ---
void showRGB(int r, int g, int b);
void showStressed();
void showNormal();
void showHappy();

void setup()
{
  Serial.begin(115200);
  pinMode(HEART_LED_PIN, OUTPUT);

  // --- ADDED: Initialize LED pins ---
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // 创建 FreeRTOS 对象
  motionDetectedSemaphore = xSemaphoreCreateBinary();
  fallDetectedSemaphore = xSemaphoreCreateBinary();
  i2cMutex = xSemaphoreCreateMutex(); // 互斥锁必须创建
  adcMutex = xSemaphoreCreateMutex();
  sensorDataMutex = xSemaphoreCreateMutex(); // 共享数据互斥锁

  // 如果任何一个信号量创建失败，打印错误并停止
  if (motionDetectedSemaphore == NULL || fallDetectedSemaphore == NULL || i2cMutex == NULL || sensorDataMutex == NULL || adcMutex == NULL)
  {
    Serial.println("FATAL: FreeRTOS object creation failed!");
    while (1)
      delay(1000);
  }

  dht.begin();
  Wire.begin();

  // ----------------------------------------------------
  // --- MPU/OLED 初始化: I2C MUX ---
  // ----------------------------------------------------

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE)
  {

    if (mpu.begin())
    {
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
    else
    {
      Serial.println("Failed to find MPU6050 chip");
    }

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
      Serial.println(F("SSD1306 allocation failed"));
    }
    else
    {
      Serial.println(F("SSD1306 display initialized"));
      display.clearDisplay();
      display.display();
    }
    // --- OLED Init ---
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is the common address
        Serial.println(F("FATAL: SSD1306 allocation failed"));
    } else {
        Serial.println(F("OLED display initialized"));
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0,0);
        display.println("Waiting...");
        display.display();
    }
    xSemaphoreGive(i2cMutex);
  }
  else
  {
    Serial.println("Setup: Failed to get I2C Mutex for init!");
  }

  // MPU6050 中断配置（使用加锁的 mpu_write_register）
  mpu_write_register(MPU6050_INT_PIN_CFG, MPU_INT_CONFIG);
  mpu_write_register(MPU6050_INT_ENABLE, MPU_INT_ENABLE_MOT_EN); // 0x40 = MOT_EN
  mpu_write_register(MPU6050_MOT_THR, 20);
  mpu_write_register(MPU6050_MOT_DUR, 1);

  // ESP32 外部中断配置
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), MPU_ISR, RISING); // INT 引脚拉高时触发

  // --- 连接 Wi-Fi ---
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // --- MQTT Setup ---
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback); // <-- ADDED: Set the callback function

  // 创建任务
  // Core 1 (实时传感器)
  xTaskCreatePinnedToCore(Task_LDR, "LDR", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_HeartRate, "HeartRate", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_DHT, "DHT", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Sound, "Sound", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Motion, "MotionTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Task_Alarm, "AlarmTask", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_OLED, "OLEDdisplay", 2048, NULL, 3, NULL, 1);

  // Core 0 (Connectivity & Status)
  xTaskCreatePinnedToCore(Task_UploadData, "Upload", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(Task_MQTT_Loop, "MQTTLoop", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(Task_LED_Control, "LEDs", 2048, NULL, 1, NULL, 0); // <-- ADDED: Start LED task
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// --- NEW Task_OLED (Displays Text and Score) ---
void Task_OLED(void *pvParameters)
{
  // Static variables to track the last-drawn state
  static LedState oledLastState = (LedState)-1; // -1 forces a redraw on first run
  static float oledLastScore = -1.0;

  while (1)
  {
    // Copy volatile variables to local copies for safe comparison
    LedState currentStateCopy = g_currentState;
    float currentScoreCopy = g_happiness_score;

    // Check if the state OR the score has changed
    if (currentStateCopy != oledLastState || currentScoreCopy != oledLastScore)
    {
      // State has changed, we need to redraw.
      // Get the I2C lock since OLED and MPU share the bus
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        display.clearDisplay(); // Clear the screen

        String emotionText;
        
        // 1. Select the text based on the state
        switch (currentStateCopy)
        {
        case STATE_HAPPY:
          emotionText = "HAPPY";
          break;
        case STATE_STRESSED:
          emotionText = "STRESSED";
          break;
        case STATE_NORMAL:
        default:
          emotionText = "NORMAL";
          break;
        }

        // --- Draw the text (centered) ---
        
        // 2. Draw Emotion Text (Size 1)
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        // Center text: (Screen_Width - (Num_Chars * 6)) / 2
        int16_t x1 = (display.width() - (emotionText.length() * 6)) / 2;
        display.setCursor(x1, 10); // Y=10
        display.println(emotionText);

        // 3. Draw Score Text (Size 2)
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        String scoreText = String(currentScoreCopy, 1); // Format score to 1 decimal place
        // Center text: (Screen_Width - (Num_Chars * 12)) / 2
        int16_t x2 = (display.width() - (scoreText.length() * 12)) / 2;
        display.setCursor(x2, 25); // Y=26
        display.println(scoreText);

        // Send buffer to the display
        display.display();

        // Release the I2C lock
        xSemaphoreGive(i2cMutex);

        // Update the last-drawn state
        oledLastState = currentStateCopy;
        oledLastScore = currentScoreCopy;
      }
      else
      {
        Serial.println("Task_OLED: Failed to get I2C Mutex!");
      }

    } // end if (state changed)

    // Sleep for 100ms before checking again
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// --- MQTT Callback Function (FIXED) ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to a String
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.printf("MQTT Callback: Topic [%s]: %s\n", topic, message.c_str());

  // Check if the message is on the command topic
  if (String(topic) == mqtt_command_topic) {
    
    // --- ADDED: Parse the "Emotion:Score" format ---
    int separatorIndex = message.indexOf(':');
    
    // Check if the separator was found
    if (separatorIndex != -1) {
      String emotion = message.substring(0, separatorIndex);
      String scoreStr = message.substring(separatorIndex + 1);
      float happinessScore = scoreStr.toFloat();

      Serial.printf("Received Emotion: %s, Score: %.1f\n", emotion.c_str(), happinessScore);

      // Store the score and state globally
      g_happiness_score = happinessScore;

      // --- Use the 'emotion' string for state logic ---
      if (emotion == "Sad") { 
        g_currentState = STATE_STRESSED; // Map "Sad" to "Stressed" LED
        Serial.println("State set to STRESSED (from 'Sad')");
      } 
      else if (emotion == "Happy") {
        g_currentState = STATE_HAPPY;
        Serial.println("State set to HAPPY");
      }
      else if (emotion == "Normal") {
        g_currentState = STATE_NORMAL;
        Serial.println("State set to NORMAL");
      }
    } else {
      Serial.println("Error: Received message in unknown format (expected 'Emotion:Score')");
    }
  }
}
void reconnectMQTT()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("esp32Client"))
    {
      Serial.println("connected");
      // --- ADDED: Subscribe to the command topic ---
      if (client.subscribe(mqtt_command_topic))
      {
        Serial.printf("Subscribed to command topic: %s\n", mqtt_command_topic);
      }
      else
      {
        Serial.println("Failed to subscribe to command topic!");
      }
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

void Task_MQTT_Loop(void *pvParameters)
{
  while (1)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (!client.connected())
      {
        reconnectMQTT();
      }
      client.loop(); // Handle MQTT housekeeping
    }
    else
    {
      Serial.println("MQTT Loop: WiFi disconnected.");
    }
    // Check connection status frequently
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void Task_UploadData(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(16000); // 16 seconds
  TickType_t xLastWakeTime = xTaskGetTickCount();

  char payload[512];
  StaticJsonDocument<512> doc;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    float noise_avg_calculated = 0.0;

    if (WiFi.status() != WL_CONNECTED || !client.connected())
    {
      Serial.println("UploadTask: WiFi or MQTT disconnected. Skipping upload.");
      continue;
    }

    // --- 创建 JSON 对象 ---
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      doc.clear();
      doc["device_name"] = device_name;
      doc["bpm"] = g_bpm;
      doc["temperature"] = g_temp;
      doc["humidity"] = g_hum;
      // doc["noise"] = g_noise_peak;
      doc["ldr"] = g_ldr;
      doc["in_motion"] = g_in_motion;

      if (g_noise_sample_count > 0)
      {
        noise_avg_calculated = (float)(g_noise_sum / g_noise_sample_count);
      }

      doc["noise"] = noise_avg_calculated;
      g_noise_peak = 0;
      g_noise_sum = 0.0;
      g_noise_sample_count = 0;
      g_in_motion = 0;

      xSemaphoreGive(sensorDataMutex);
    }
    else
    {
      Serial.println("UploadTask: Failed to get sensor mutex. Skipping.");
      continue;
    }

    serializeJson(doc, payload, sizeof(payload));

    Serial.print("Uploading payload: ");
    Serial.println(payload);

    if (!client.publish(mqtt_topic, payload))
    {
      Serial.println("MQTT Publish failed!");
    }
  }
}

// 写入MPU
void mpu_write_register(uint8_t reg_addr, uint8_t data)
{
  // 保护 I2C
  // 注意：如果 Mutex 未在 setup() 中创建，这里会崩溃
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg_addr);
    Wire.write(data);
    if (Wire.endTransmission() != 0)
    {
      Serial.println("MPU Write Failed!");
    }
    xSemaphoreGive(i2cMutex);
  }
  else
  {
    Serial.println("MPU Write: Failed to get I2C Mutex!");
  }
}

// --- ADDED: LED Helper Functions (from your snippet) ---

// Helper function to set all 3 colors
void showRGB(int r, int g, int b)
{
  // Constrain values to be 0-255
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);

  // Use the #defined pins
  analogWrite(LED_RED, r);
  analogWrite(LED_GREEN, g);
  analogWrite(LED_BLUE, b);
}

// STATE 1: "Stressed"
void showStressed()
{
  if (millis() - stressedLastUpdate > stressedBlinkSpeed)
  {
    stressedLastUpdate = millis();
    stressedState = !stressedState;

    if (stressedState)
    {
      showRGB(255, 0, 0); // LED On (Bright Red)
    }
    else
    {
      showRGB(0, 0, 0); // LED Off
    }
  }
}

// STATE 2: "Normal"
void showNormal()
{
  showRGB(0, 0, 255); // Solid blue
}

// STATE 3: "Happy"
void showHappy()
{
  float pulse = (sin(millis() * happyPulseSpeed) + 1.0) / 2.0;
  int greenValue = 50 + (pulse * 205);
  showRGB(0, greenValue, 0); // Breathing green
}

// --- New FreeRTOS Task for LED Control (FIXED) ---
void Task_LED_Control(void *pvParameters)
{
  Serial.println("Task_LED_Control started.");
  while (1)
  {

    // --- THIS IS THE FIX ---
    // Change "String" to "LedState"
    // We make a local copy because the variable is volatile
    LedState currentStateCopy = g_currentState;
    // --- END OF FIX ---

    // Check the local copy and call the matching function
    if (currentStateCopy == STATE_STRESSED)
    {
      showStressed();
    }
    else if (currentStateCopy == STATE_HAPPY)
    {
      showHappy();
    }
    else
    {
      // Default to "normal"
      showNormal();
    }

    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(20)); // Update ~50 times/sec
  }
}

// ---------------------- 任务定义 ----------------------

// 心率计数
void Task_HeartRate(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(HEART_PERIOD_MS); // 50ms 周期
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // 静态变量：保持状态，只在任务启动时初始化
  static float oldValue = 500.0;
  static float thresholdMax = 0.0;
  static unsigned long bpmMills = 0;
  static int bpm_count = 0;
  static unsigned long FirstBpmTime = 0;
  static unsigned long timeBetweenBeats = 0;

  // 确保时间变量在第一次运行时被初始化
  if (bpmMills == 0)
  {
    bpmMills = millis();
    FirstBpmTime = millis();
    timeBetweenBeats = millis();
  }

  int minDelayBetweenBeats = 400;

  while (1)
  {
    int rawValue = analogRead(HEART_PIN);

    // 1. 低通滤波 (平滑信号)
    float value = ALPHA * oldValue + (1 - ALPHA) * rawValue;
    float change = value - oldValue;
    oldValue = value;

    // 2. 无手指检测 (2秒无变化则重置BPM)
    if (change < MIN_CHANGE && (millis() - timeBetweenBeats) >= 2000)
    {
      bpm_count = 0;
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        g_bpm = 0.0; // 报告 BPM 为 0
        xSemaphoreGive(sensorDataMutex);
      }
    }

    // 3. 脉冲峰值检测 (动态阈值)
    if ((change >= thresholdMax) &&
        (change > MIN_CHANGE) &&
        (change < 30) && // 限制最大变化值以过滤尖锐噪声
        (millis() > timeBetweenBeats + minDelayBetweenBeats))
    {

      // 首次心跳记录时间
      if (bpm_count == 0)
      {
        FirstBpmTime = millis();
      }

      // 重置动态阈值
      thresholdMax = change;
      digitalWrite(HEART_LED_PIN, HIGH);

      // 更新心跳时间
      timeBetweenBeats = millis();
      bpm_count++;
      Serial.print("Heart beat detected!\n");
    }
    else
    {
      digitalWrite(HEART_LED_PIN, LOW);
    }

    // 4. 阈值衰减
    thresholdMax = thresholdMax * 0.97;

    // 5. BPM 周期性报告 (15秒窗口)
    if (millis() >= bpmMills + 15000)
    {
      float calculated_bpm = 0.0;
      if (bpm_count > 0)
      {
        calculated_bpm = (float)bpm_count * (60000.0 / (timeBetweenBeats - FirstBpmTime));
      }

      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        g_bpm = calculated_bpm;
        xSemaphoreGive(sensorDataMutex);
      }

      Serial.printf("BPM (approx): %.1f\n", calculated_bpm);
      bpm_count = 0;
      bpmMills = millis();
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 湿度温度
void Task_DHT(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(29000); // 每29s
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      g_temp = t;
      g_hum = h;
      xSemaphoreGive(sensorDataMutex);
    }

    Serial.printf("Temp: %.1f C, Hum: %.1f %%\n", t, h);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void Task_LDR(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(20000); // 20 seconds
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    float l = 0.0; // Default to 0

    if (xSemaphoreTake(adcMutex, pdMS_TO_TICKS(60)) == pdTRUE)
    {
      l = analogRead(LDR_PIN);
      xSemaphoreGive(adcMutex);
    }
    else
    {
      Serial.println("LDR Task: Failed to get ADC Mutex!");
      l = -1; // Or some other error value
    }
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
      g_ldr = l;
      xSemaphoreGive(sensorDataMutex);
    }

    Serial.printf("light: %.1f lu\n", l);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 噪声监测
void Task_Sound(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(200);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    unsigned long startTime = millis();
    int signalMax = 0;
    int signalMin = 1023;

    while (millis() - startTime < SAMPLE_SIZE_S)
    {
      int sample = analogRead(SOUND_PIN);
      if (sample < 1024)
      {
        if (sample > signalMax)
          signalMax = sample;
        else if (sample < signalMin)
          signalMin = sample;
      }
    }
    int amplitude = signalMax - signalMin; // 当前振幅 (每 200ms)

    // 2. 更新全局聚合变量 (使用 Mutex 保护)
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      // 更新峰值
      if (amplitude > g_noise_peak)
      {
        g_noise_peak = amplitude;
      }
      // 累加平均值
      g_noise_sum += (double)amplitude;
      g_noise_sample_count++;
      xSemaphoreGive(sensorDataMutex);
    }
    else
    {
      Serial.println("Task_Sound: Failed to get sensor mutex!");
    }

    // debug
    // Serial.printf("Noise amplitude: %d\n", amplitude);

    if (amplitude > NOISE_THRESHOLD)
    {
      Serial.println("Loud sound detected!");
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 动作监测
void Task_Motion(void *pvParameters)
{
  sensors_event_t a, g, temp;
  for (;;)
  {
    // 阻塞等待 MPU 中断信号
    if (xSemaphoreTake(motionDetectedSemaphore, portMAX_DELAY) == pdTRUE)
    {

      // MPU6050 中断被触发，立即读取数据和中断状态 (I2C 操作必须加锁)
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
      { // 获取 I2C 锁
        mpu.getEvent(&a, &g, &temp);

        // 关键修复: 读取中断状态寄存器 (0x3A) 以清除 MPU6050 的 INT 锁存。
        // 这一操作必须在 Mutex 保护下进行。
        uint8_t status;
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(MPU6050_INT_STATUS);
        Wire.endTransmission(false); // 发送重启条件
        Wire.requestFrom(MPU6050_ADDRESS, 1);
        if (Wire.available())
        {
          status = Wire.read();
          Serial.printf("MPU INT Status Cleared: 0x%X\n", status);
        }
        else
        {
          Serial.println("Warning: Failed to read MPU INT Status!");
        }

        xSemaphoreGive(i2cMutex); // 释放锁
      }
      else
      {
        Serial.println("MotionTask: Failed to get I2C Mutex!");
        // 如果获取锁失败，继续等待下一次信号量
        continue;
      }

      // 注意：getEvent() 之后才能进行计算
      float accTotal = sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z);

      if (accTotal > 10.0)
      { // 检测到高 G 撞击
        Serial.println("IMPACT DETECTED after MPU Interrupt!");
        xSemaphoreGive(fallDetectedSemaphore);
        if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(200)) == pdTRUE)
        {
          g_in_motion = 1; // 设置标志
          xSemaphoreGive(sensorDataMutex);
        }
        else
        {
          Serial.println("MotionTask: Failed to get sensor mutex for status update!");
        }
      }
      else if (accTotal < 3.0)
      {
        Serial.println("Potential Free Fall (Low G) detected via Interrupt.");
      }
    }
  }
}

// 摔倒警报
void Task_Alarm(void *pvParameters)
{
  for (;;)
  {
    // 等待中断信号
    if (xSemaphoreTake(fallDetectedSemaphore, portMAX_DELAY) == pdTRUE)
    {
      Serial.println("ALARM TRIGGERED!");
      //       digitalWrite(buzzerPin, HIGH);
      vTaskDelay(pdMS_TO_TICKS(2000));
      //       digitalWrite(buzzerPin, LOW);
    }
  }
}

// 中断服务程序 (ISR)
void IRAM_ATTR MPU_ISR()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // 仅给出信号量，不在 ISR 中执行 I2C 操作
  xSemaphoreGiveFromISR(motionDetectedSemaphore, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE)
  {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 立即进行上下文切换
  }
}