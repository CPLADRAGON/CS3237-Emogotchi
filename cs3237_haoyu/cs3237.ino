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
#define HEART_PIN 32
#define SOUND_PIN 34
//#define buzzerPin 8 // 警报信号
#define MPU_INT_PIN 4 // MPU6050 INT 引脚

// 变量
#define SAMPLE_SIZE_H 4     // 心率，4次采样
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

  // ----------------------------------------------------
  // --- 关键修复：创建 FreeRTOS 对象 ---
  // ----------------------------------------------------
  motionDetectedSemaphore = xSemaphoreCreateBinary();
  fallDetectedSemaphore = xSemaphoreCreateBinary();
  i2cMutex = xSemaphoreCreateMutex(); // 互斥锁必须创建

  // 如果任何一个信号量创建失败，打印错误并停止
  if (motionDetectedSemaphore == NULL || fallDetectedSemaphore == NULL || i2cMutex == NULL) {
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
  mpu_write_register(MPU6050_INT_ENABLE, MPU_INT_ENABLE_MOT_EN); // 0x40 = MOT_EN

  // ESP32 外部中断配置
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), MPU_ISR, RISING); // INT 引脚拉高时触发

  // ----------------------------------------------------

  // 创建任务
  xTaskCreatePinnedToCore(Task_HeartRate, "HeartRate", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_DHT, "DHT", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Sound, "Sound", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Motion, "MotionTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Task_Alarm, "AlarmTask", 2048, NULL, 3, NULL, 1);
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
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 每10ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  float reads[SAMPLE_SIZE_H] = {0}, sum = 0, before = 0;
  int ptr = 0, rise_count = 0;
  bool rising = false;
  unsigned long lastBeatTime = 0;
  float first = 0, second = 0, third = 0;

  while (1) {
    float reader = analogRead(HEART_PIN);

    // 平滑心率（4次平均）
    sum -= reads[ptr];
    sum += reader;
    reads[ptr] = reader;
    ptr = (ptr + 1) % 4;
    float avgValue = sum / 4;

    // 检测上升边沿（5个心跳峰）
    if (avgValue > before) {
      rise_count++;
      if (!rising && rise_count > 5) {
        rising = true;
        unsigned long now = millis();
        if (lastBeatTime > 0) {
          // 心跳间隔与 BPM 计算
          float interval = now - lastBeatTime;
          float avgInterval = (0.4 * interval + 0.3 * second + 0.3 * third);
          float bpm = 60000.0 / avgInterval;
          Serial.print("BPM: ");
          Serial.println(bpm, 1);
          third = second;
          second = interval;
        }
        lastBeatTime = now;
      }
    } else {
      rising = false;
      rise_count = 0;
    }
    before = avgValue;

    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // 精准定时
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
      int sample = analogRead(SOUND_PIN);
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
          Wire.requestFrom(MPU6050_ADDRESS, 1);
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