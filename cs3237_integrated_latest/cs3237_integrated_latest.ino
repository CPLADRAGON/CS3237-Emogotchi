#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include "Wire.h"
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cmath> 
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// PIN
#define DHTPIN 25
#define DHTTYPE DHT11
#define HEART_PIN 32
#define SOUND_PIN 34
//#define buzzerPin 8 // 警报信号
#define MPU_INT_PIN 4 // MPU6050 INT 引脚
#define HEART_LED_PIN 2 // 新增: 脉搏指示灯（假设为 ESP32 内置 LED）
#define SCREEN_WIDTH 64 // OLED 宽度
#define SCREEN_HEIGHT 48 // OLED 高度

// 变量
#define SAMPLE_SIZE_H 4     // 心率，4次采样
#define SAMPLE_SIZE_S 50    // 噪音，50ms 采样窗口
#define NOISE_THRESHOLD 500
#define MIN_CHANGE 7.5
#define ALPHA 0.75
#define HEART_PERIOD_MS 50 // 采样周期 50ms

// --- WiFi & HTTP 常量 ---
const char* ssid = "CPLADRAGON";
const char* password = "10293847";
const char* serverUrl = "http://172.20.10.4:5000/api/upload"; 

// MPU6050 寄存器地址和配置
#define MPU6050_ADDRESS       0x68
#define MPU6050_INT_PIN_CFG   0x37
#define MPU6050_INT_ENABLE    0x38
#define MPU6050_INT_STATUS    0x3A // 中断状态寄存器地址
#define MPU_INT_CONFIG        0x20 // 0x37 寄存器值：高电平有效，推挽输出，锁存直到读取
#define MPU_INT_ENABLE_MOT_EN 0x40 // 0x38 寄存器值：只启用运动检测中断

// 对象化
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); 
SemaphoreHandle_t fallDetectedSemaphore;
SemaphoreHandle_t motionDetectedSemaphore;
SemaphoreHandle_t i2cMutex; // I2C 互斥锁
SemaphoreHandle_t sensorDataMutex; // OLED 互斥锁
SemaphoreHandle_t adcMutex; // <-- FIX 1: Re-add the ADC mutex

// 共享传感器数据
float g_bpm = 0.0;
float g_temp = 0.0;
float g_hum = 0.0;
int g_noise = 0;

// 函数声明
void Task_HeartRate(void *pvParameters);
void Task_DHT(void *pvParameters);
void Task_Sound(void *pvParameters);
void Task_Motion(void *pvParameters); 
void Task_Alarm(void *pvParameters);
void IRAM_ATTR MPU_ISR();
void mpu_write_register(uint8_t reg_addr, uint8_t data);
void Task_OLED(void *pvParameters);
void Task_UploadData(void *pvParameters);




void setup() {
  Serial.begin(115200);
  pinMode(HEART_LED_PIN, OUTPUT);
//  pinMode(buzzerPin, OUTPUT);

  // ----------------------------------------------------
  // --- 关键修复：创建 FreeRTOS 对象 ---
  // ----------------------------------------------------
  motionDetectedSemaphore = xSemaphoreCreateBinary();
  fallDetectedSemaphore = xSemaphoreCreateBinary();
  i2cMutex = xSemaphoreCreateMutex(); // 互斥锁必须创建
  sensorDataMutex = xSemaphoreCreateMutex(); // 共享数据互斥锁
  adcMutex = xSemaphoreCreateMutex(); // <-- FIX 1: Create the ADC mutex

  // 如果任何一个信号量创建失败，打印错误并停止
  // <-- FIX 1: Add adcMutex to the check
  if (motionDetectedSemaphore == NULL || fallDetectedSemaphore == NULL || i2cMutex == NULL || sensorDataMutex == NULL || adcMutex == NULL) {
      Serial.println("FATAL: FreeRTOS object creation failed!");
      while (1) delay(1000); 
  }
  // ----------------------------------------------------

  dht.begin();
  Wire.begin(); 

  // ----------------------------------------------------
  // --- MPU/OLED 初始化: I2C MUX ---
  // ----------------------------------------------------
  
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    if(display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println("System Ready!");
      display.display();
    } else {
      Serial.println("SSD1306 allocation failed");
    }
    
    if (mpu.begin()) {
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    } else {
       Serial.println("Failed to find MPU6050 chip");
    }
    xSemaphoreGive(i2cMutex);
  } else {
    Serial.println("Setup: Failed to get I2C Mutex for init!");
  }


  // MPU6050 中断配置（使用加锁的 mpu_write_register）
  mpu_write_register(MPU6050_INT_PIN_CFG, MPU_INT_CONFIG); 
  mpu_write_register(MPU6050_INT_ENABLE, MPU_INT_ENABLE_MOT_EN); // 0x40 = MOT_EN

  // ESP32 外部中断配置
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), MPU_ISR, RISING); // INT 引脚拉高时触发

  // ----------------------------------------------------

  // --- 连接 Wi-Fi ---
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // 创建任务
  // Core 1 (实时传感器)
  xTaskCreatePinnedToCore(Task_HeartRate, "HeartRate", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_DHT, "DHT", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Sound, "Sound", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Motion, "MotionTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Task_Alarm, "AlarmTask", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_OLED, "OLEDTask", 4096, NULL, 4, NULL, 1);
  // Core 0 (网络通信)
  xTaskCreatePinnedToCore(Task_UploadData, "UploadTask", 4096, NULL, 4, NULL, 0);
}


void loop() {

}

void mpu_write_register(uint8_t reg_addr, uint8_t data) {
    // 保护 I2C
    // 注意：如果 Mutex 未在 setup() 中创建，这里会崩溃
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) { 
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(reg_addr);
        Wire.write(data);
        if (Wire.endTransmission() != 0) {
            Serial.println("MPU Write Failed!");
        }
        xSemaphoreGive(i2cMutex); 
    } else {
        Serial.println("MPU Write: Failed to get I2C Mutex!");
    }
}

// ---------------------- 任务定义 ----------------------

// 心率计数
void Task_HeartRate(void *pvParameters) {
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
    if (bpmMills == 0) {
        bpmMills = millis();
        FirstBpmTime = millis();
        timeBetweenBeats = millis();
    }

    int minDelayBetweenBeats = 400;

    while (1) {
        // <-- FIX 1: Declare rawValue here
        int rawValue; 
        
        // <-- FIX 1: Protect ADC read with mutex
        if (xSemaphoreTake(adcMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            rawValue = analogRead(HEART_PIN);
            xSemaphoreGive(adcMutex);
        } else {
            Serial.println("Heart task failed to get ADC mutex");
            rawValue = oldValue; // Use old value on failure
        }

        // 1. 低通滤波 (平滑信号)
        float value = ALPHA * oldValue + (1 - ALPHA) * rawValue;
        float change = value - oldValue;
        oldValue = value;

        // 2. 无手指检测 (2秒无变化则重置BPM)
        if (change < MIN_CHANGE && (millis() - timeBetweenBeats) >= 2000) {
          bpm_count = 0;
          if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
              g_bpm = 0.0; // 报告 BPM 为 0
              xSemaphoreGive(sensorDataMutex);
          }
        }

        // 3. 脉冲峰值检测 (动态阈值)
        if ((change >= thresholdMax) && 
            (change > MIN_CHANGE) && 
            (change < 30) && // 限制最大变化值以过滤尖锐噪声
            (millis() > timeBetweenBeats + minDelayBetweenBeats)) {
            
            // 首次心跳记录时间
            if (bpm_count == 0){
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
        else {
            digitalWrite(HEART_LED_PIN, LOW);
        }

        // 4. 阈值衰减
        thresholdMax = thresholdMax * 0.97;

        // 5. BPM 周期性报告 (15秒窗口)
        if (millis() >= bpmMills + 15000) {
            float calculated_bpm = 0.0;
            if (bpm_count > 0) {
                // 计算平均 BPM
                calculated_bpm = (float)bpm_count * (60000.0 / (timeBetweenBeats - FirstBpmTime)); 
            }
            
            // 安全写入共享数据
            if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_bpm = calculated_bpm;
                xSemaphoreGive(sensorDataMutex);
            }

            Serial.printf("BPM (approx): %.1f\n", calculated_bpm);
            
            // 重置计数器和时间戳
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
    
    // --- FIX: ADD THIS BLOCK ---
    // Safely update the global variables
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (!isnan(t)) g_temp = t; // Only update if the reading was valid
      if (!isnan(h)) g_hum = h;
      xSemaphoreGive(sensorDataMutex);
    } // <-- FIX 2: Mutex is released here

    // <-- FIX 2: Delay is now OUTSIDE the mutex block
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
      
      // <-- FIX 1: Protect ADC read with mutex
      if (xSemaphoreTake(adcMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          sample = analogRead(SOUND_PIN);
          xSemaphoreGive(adcMutex);
      } else {
          Serial.println("Sound task failed to get ADC mutex");
          sample = 1023; // Default value
      }
      
      if (sample < 1024) {
        if (sample > signalMax) signalMax = sample;
        else if (sample < signalMin) signalMin = sample;
      }
    }

    int amplitude = signalMax - signalMin;
    //Serial.printf("Noise amplitude: %d\n", amplitude);

    if (amplitude > NOISE_THRESHOLD) {
      Serial.println("Loud sound detected!");
    }
    
    // --- FIX: ADD THIS BLOCK ---
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      g_noise = amplitude;
      xSemaphoreGive(sensorDataMutex);
    } // <-- FIX 2: Mutex is released here

    // <-- FIX 2: Delay is now OUTSIDE the mutex block
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// OLED显示
void Task_OLED(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 每500ms
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    float temp_bpm = 0.0;
    float temp_t = 0.0;
    float temp_h = 0.0;
    int temp_n = 0;
    
    char lineBuffer[16];

    while (1) {
        // 1. 读取共享数据 (必须使用 sensorDataMutex 保护)
        if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            temp_bpm = g_bpm;
            temp_t = g_temp;
            temp_h = g_hum;
            temp_n = g_noise;
            xSemaphoreGive(sensorDataMutex);
        }

        // 2. 刷新 OLED 屏幕 (必须使用 i2cMutex 保护)
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            display.clearDisplay();
            display.setCursor(0, 0);

            // Line 1: Temp & Hum
            snprintf(lineBuffer, sizeof(lineBuffer), "T:%.1f H:%.0f%%", temp_t, temp_h);
            display.println(lineBuffer);

            // Line 2: BPM
            snprintf(lineBuffer, sizeof(lineBuffer), "BPM:%.0f", temp_bpm);
            display.println(lineBuffer);

            // Line 3: Noise
            snprintf(lineBuffer, sizeof(lineBuffer), "Noise:%d", temp_n);
            display.println(lineBuffer);

            // Line 4: Status (Placeholder for fall/motion status)
            // Note: MPU status logic needs shared variable update in Task_Motion if needed here.
            display.println("Status: OK"); 

            display.display();
            xSemaphoreGive(i2cMutex);
        } else {
            Serial.println("OLED Task: Failed to get I2C Mutex!");
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);   // 周期性延迟
    }
}

// 动作监测
void Task_Motion(void *pvParameters) {
  sensors_event_t a, g, temp;
  for (;;) {
    // 阻塞等待 MPU 中断信号
    if (xSemaphoreTake(motionDetectedSemaphore, portMAX_DELAY) == pdTRUE) {
      
      // MPU6050 中断被触发，立即读取数据和中断状态 (I2C 操作必须加锁)
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) { // 获取 I2C 锁
          mpu.getEvent(&a, &g, &temp);
          
          // 关键修复: 读取中断状态寄存器 (0x3A) 以清除 MPU6050 的 INT 锁存。
          // 这一操作必须在 Mutex 保护下进行。
          uint8_t status;
          Wire.beginTransmission(MPU6050_ADDRESS);
          Wire.write(MPU6050_INT_STATUS);
          Wire.endTransmission(false); // 发送重启条件
          Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)1);
          if (Wire.available()) {
              status = Wire.read();
              Serial.printf("MPU INT Status Cleared: 0x%X\n", status);
          } else {
              Serial.println("Warning: Failed to read MPU INT Status!");
          }

          xSemaphoreGive(i2cMutex); // 释放锁
      } else {
          Serial.println("MotionTask: Failed to get I2C Mutex!");
          // 如果获取锁失败，继续等待下一次信号量
          continue; 
      }
      
      // 注意：getEvent() 之后才能进行计算
      float accTotal = sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z);

      if (accTotal > 10.0) { // 检测到高 G 撞击
        Serial.println("IMPACT DETECTED after MPU Interrupt!");
        xSemaphoreGive(fallDetectedSemaphore);
      } else if (accTotal < 3.0) {
          Serial.println("Potential Free Fall (Low G) detected via Interrupt.");
      }
    }
  }
}

// 摔倒警报
void Task_Alarm(void *pvParameters) {
  for (;;) {
    // 等待中断信号
    if (xSemaphoreTake(fallDetectedSemaphore, portMAX_DELAY) == pdTRUE) {
      Serial.println("ALARM TRIGGERED!");
//      digitalWrite(buzzerPin, HIGH);
      vTaskDelay(pdMS_TO_TICKS(2000));
//      digitalWrite(buzzerPin, LOW);
    }
  }
}

// 中断服务程序 (ISR)
void IRAM_ATTR MPU_ISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // 仅给出信号量，不在 ISR 中执行 I2C 操作
  xSemaphoreGiveFromISR(motionDetectedSemaphore, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 立即进行上下文切换
  }
}

//数据上传
  // ----------------------------------------------------
  // --- 只读取每30s瞬间的global variable ---
  // ----------------------------------------------------
void Task_UploadData(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(30000); 
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency); //每30s

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("UploadTask: WiFi disconnected. Skipping upload.");
      continue; 
    }

    // 准备 JSON 数据
    StaticJsonDocument<256> doc; // 创建一个 JSON 文档
    String payload; // 用于存储序列化后的 JSON 字符串

    // 从全局变量读取数据
  // ----------------------------------------------------
  // --- 只读取每30s瞬间的global variable ---
  // ----------------------------------------------------
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        doc["bpm"] = g_bpm;
        doc["temperature"] = g_temp;
        doc["humidity"] = g_hum;
        doc["noise"] = g_noise;
        xSemaphoreGive(sensorDataMutex); // 立即释放
    } else {
        Serial.println("UploadTask: Failed to get data mutex. Skipping upload.");
        continue;
    }

    //将 JSON 文档序列化为字符串
    serializeJson(doc, payload);
    Serial.print("Uploading payload: ");
    Serial.println(payload);

    //发起 HTTP POST 请求
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json"); 
    int httpCode = http.POST(payload);

    //检查服务器响应
    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) { // HTTP 200
        String response = http.getString();
        Serial.println("Upload successful! Server response:");
        Serial.println(response);
      } else {
        Serial.printf("Upload failed, HTTP error code: %d\n", httpCode);
      }
    } else {
      Serial.printf("Upload failed, HTTPClient error: %s\n", http.errorToString(httpCode).c_str());
    }

    //释放 HTTPClient 资源
    http.end();   //结束对话
  }
}