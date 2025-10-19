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

// PIN
#define DHTPIN 25
#define DHTTYPE DHT11
// --- NEW PULSE SENSOR PINS ---
#define ledPin 2      // Using built-in LED pin for heartbeat indication
#define sensorPin 32  // Pulse meter connected to an Analog pin
// --- END NEW PINS ---
#define SOUND_PIN 34
//#define buzzerPin 8 // 警报信号
#define MPU_INT_PIN 4 // MPU6050 INT 引脚

// 变量
// --- NEW PULSE SENSOR VARS ---
#define MIN_CHANGE 3.0      // Adjusted threshold for higher sensitivity
float thresholdMax = 0.0;
float alpha = 0.75;         // Values for the filter
int period = 50;            // loop delay
// --- END NEW VARS ---
#define SAMPLE_SIZE_S 50    // 噪音，50ms 采样窗口
#define NOISE_THRESHOLD 500

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
// Adafruit_SSD1306 display(64, 48, &Wire, -1); 

SemaphoreHandle_t fallDetectedSemaphore;
SemaphoreHandle_t motionDetectedSemaphore;
SemaphoreHandle_t i2cMutex; // I2C 互斥锁
SemaphoreHandle_t adcMutex; // ADC 互斥锁

// 函数声明
void Task_HeartRate(void *pvParameters);
void Task_DHT(void *pvParameters);
void Task_Sound(void *pvParameters);
void Task_Motion(void *pvParameters); 
void Task_Alarm(void *pvParameters);
void IRAM_ATTR MPU_ISR();
void mpu_write_register(uint8_t reg_addr, uint8_t data);

void setup() {
  Serial.begin(115200);
//  pinMode(buzzerPin, OUTPUT);
  
  // --- ADDED FROM NEW PULSE CODE ---
  pinMode(ledPin, OUTPUT); // Inbuilt LED
  Serial.println("Pulse rate detection started.");
  // ---------------------------------

  // ----------------------------------------------------
  // --- 关键修复：创建 FreeRTOS 对象 ---
  // ----------------------------------------------------
  motionDetectedSemaphore = xSemaphoreCreateBinary();
  fallDetectedSemaphore = xSemaphoreCreateBinary();
  i2cMutex = xSemaphoreCreateMutex(); // 互斥锁必须创建
  adcMutex = xSemaphoreCreateMutex(); 

  // 如果任何一个信号量创建失败，打印错误并停止
  if (motionDetectedSemaphore == NULL || fallDetectedSemaphore == NULL || i2cMutex == NULL || adcMutex == NULL) {
      Serial.println("FATAL: FreeRTOS object creation failed!");
      while (1) delay(1000); 
  }
  // ----------------------------------------------------

  dht.begin();
  Wire.begin();
  
  // ----------------------------------------------------
  // --- MPU 初始化：加锁保护 I2C 操作 ---
  // ----------------------------------------------------
  
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) { // 获取 I2C 锁
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      // 注意：这里需要释放锁，否则 loop() 之外的操作会永久持有锁
      xSemaphoreGive(i2cMutex);
      while (1) {
        delay(10);
      }
    }
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    // 释放锁。后续的 mpu_write_register 会自行加锁和解锁。
    xSemaphoreGive(i2cMutex); 

  } else {
    Serial.println("Setup: Failed to get I2C Mutex for MPU init!");
  }


  // MPU6050 中断配置（使用加锁的 mpu_write_register）
  mpu_write_register(MPU6050_INT_PIN_CFG, MPU_INT_CONFIG); 
  mpu_write_register(MPU6050_INT_ENABLE, MPU_INT_ENABLE_MOT_EN);
  // 0x40 = MOT_EN

  // ESP32 外部中断配置
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), MPU_ISR, RISING);
  // INT 引脚拉高时触发
  // ----------------------------------------------------
  
  // 创建任务
  xTaskCreatePinnedToCore(Task_HeartRate, "HeartRate", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_DHT, "DHT", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Sound, "Sound", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Motion, "MotionTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Task_Alarm, "AlarmTask", 2048, NULL, 3, NULL, 1);
}

void loop() {
  // Loop is empty because FreeRTOS tasks are managing everything.
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

// ------------------------------------------------------------
// REPLACED Task_HeartRate with Pulse_Detector_KY_039 logic
// ------------------------------------------------------------
void Task_HeartRate(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(period);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  static float oldValue = 500;
  static unsigned long bpmMills = millis();
  static int bpm = 0;
  static unsigned long FirstBpmTime = millis();
  static unsigned long timeBetweenBeats = millis();
  int minDelayBetweenBeats = 400;
  
  while (1) {
    int rawValue; 
    
    if (xSemaphoreTake(adcMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      rawValue = analogRead(sensorPin);
      xSemaphoreGive(adcMutex);
    } else {
      Serial.println("Heart task failed to get ADC mutex");
      rawValue = oldValue; 
    }
    
    float value = alpha * oldValue + (1 - alpha) * rawValue;
    float change = value - oldValue;
    oldValue = value;
    
    //Serial.printf("Raw: %d, Value: %.2f, Change: %.2f, T_Max: %.2f\n", rawValue, value, change, thresholdMax); // for debugging
    
    if (change < MIN_CHANGE && (millis() - timeBetweenBeats) >= 2000) {
      bpm = 0;
    }

    if ((change >= thresholdMax) && (change > MIN_CHANGE)  && (change < 30) && (millis() > timeBetweenBeats + minDelayBetweenBeats)) {
      if (bpm == 0){
        FirstBpmTime = millis();
      }

      thresholdMax = change;
      digitalWrite(ledPin, HIGH);
      timeBetweenBeats = millis();
      bpm++;
      Serial.print("Heart beat detected!\n");
    }
    else {
      digitalWrite(ledPin, LOW);
    }

    thresholdMax = thresholdMax * 0.97;

    if (millis() >= bpmMills + 15000) {
      Serial.print("BPM (approx): ");
      if (bpm == 0) {
        Serial.println("--");
      } else {
        Serial.println(bpm * (60000.0 / (timeBetweenBeats - FirstBpmTime)));
      }
      
      bpm = 0;
      bpmMills = millis();
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
// ------------------------------------------------------------
// END OF REPLACED TASK
// ------------------------------------------------------------


// 湿度温度
void Task_DHT(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(2000);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    Serial.printf("Temp: %.1f C, Hum: %.1f %%\n", t, h);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 噪声监测
void Task_Sound(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100); 
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

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 动作监测
void Task_Motion(void *pvParameters) {
  sensors_event_t a, g, temp;
  for (;;) {
    if (xSemaphoreTake(motionDetectedSemaphore, portMAX_DELAY) == pdTRUE) {
      
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) { 
          mpu.getEvent(&a, &g, &temp);
          
          uint8_t status;
          Wire.beginTransmission(MPU6050_ADDRESS);
          Wire.write(MPU6050_INT_STATUS);
          Wire.endTransmission(false); 
          Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)1);
          if (Wire.available()) {
              status = Wire.read();
              Serial.printf("MPU INT Status Cleared: 0x%X\n", status);
          } else {
              Serial.println("Warning: Failed to read MPU INT Status!");
          }

          xSemaphoreGive(i2cMutex);
      } else {
          Serial.println("MotionTask: Failed to get I2C Mutex!");
          continue;
      }
      
      float accTotal = sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z);
      if (accTotal > 10.0) { 
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
  xSemaphoreGiveFromISR(motionDetectedSemaphore, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}