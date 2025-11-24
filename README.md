# Emogotchi - CS3237 IoT Project (Team 06)

**Course:** CS3237 Introduction to Internet of Things (AY2025/26 Semester 1)  
**Team Members:** DONG HUAISHUO, LI ZHUOLUN, WANG BOYU, ZHANG HAOYU

---

## 1. Project Description

Students often spend long hours studying and can become unaware of accumulating environmental and physiological stress. This lack of awareness can lead to reduced productivity and long-term health issues.

**Emogotchi** is a real-time, multi-device IoT system designed to monitor a student's well-being. The system uses a multi-sensor **Main ESP32** device to capture physiological data (BPM, motion) and environmental data (temperature, humidity, noise, light).

This data is published via MQTT to a cloud server, which uses a time-series **LSTM (Long Short-Term Memory) machine learning model** to predict a "happiness score" (0-100).

**New: AI Wellness Coach**  
The system now integrates **Google Gemini 2.0 Flash AI**. It analyzes the raw sensor data and the predicted happiness score to generate context-aware, personalized wellness tips (e.g., "High noise detected, try finding a quieter spot" or "Heart rate elevated, take a deep breath").

This creates a **closed-loop feedback system**:
1.  **Emogotchi Device:** Displays current emotion ("Happy", "Normal", "Sad") on OLED and RGB LED.
2.  **Smart Home Hub:** Triggers physical actuators (servos, lights) and "Relax Mode" routines based on the user's state.
3.  **Web Dashboard:** Displays live data, 24-hour/7-day trends, and offers an **interactive AI advice modal** to coach the user back to a balanced state.
4.  **Telegram:** Sends critical alerts if the score drops below a threshold.

## 2. System Architecture

The project is built on a high-performance, real-time **Publish/Subscribe (Pub/Sub) model** using an MQTT broker.

1.  **Main ESP32 (Emogotchi Device):**
    * **Publisher/Subscriber**
    * Runs **FreeRTOS** to manage 10 concurrent tasks (sensors, display, network) across both CPU cores.
    * **Core 1 (Sensors):** Gathers data from the MPU6050 (interrupt-driven), DHT11, Pulse Sensor, Sound Sensor, and LDR.
    * **Core 0 (Network):** Manages Wi-Fi and MQTT.
    * **Publishes** a JSON payload of sensor data to the `esp32/sensor_data` topic.
    * **Subscribes** to the `esp32/prediction` topic to receive real-time commands.
    * Displays the received emotion on its OLED screen and RGB LED.

2.  **Cloud Server (DigitalOcean Droplet):**
    * **Broker & Brain**
    * Runs the **Mosquitto MQTT Broker** as the central message hub.
    * Runs a **Python Flask Server** (`server.py`):
        * **Subscribes** to `esp32/sensor_data`.
        * Collects a sequence of 10 data points.
        * Feeds the sequence into a trained **Keras/TensorFlow LSTM model** to predict a "happiness score" (0-100).
        * **Generative AI:** Calls the **Google Gemini API** to generate text-based wellness advice based on specific sensor readings.
        * **Publishes** the emotion and score back to `esp32/prediction`.
        * Sends **Telegram alerts** for critical stress levels.
        * Logs data to `sensor_data.csv` and `happiness_trend.csv`.
        * **Serves a Web UI:**
            * Live sensor metrics.
            * **Interactive Stress Card:** Clicking opens a modal with the AI's latest advice.
            * **Dynamic Charts:** Users can toggle between 24-Hour and 7-Day history views.

3.  **Smart Home Hub (ASR-PRO & Second ESP32):**
    * **Voice & Emotion Actuator Controller**
    * **Subscribes** to the `esp32/prediction` topic.
    * **ASR-PRO Module:** Serves as the primary **offline voice recognition engine** and central actuator controller.
    * **Sentiment Gateway (Second ESP32):** Acts as a bridge between the cloud and the offline hub. When a low happiness score is detected, it triggers the ASR-PRO via interrupt.
    * **Relax Mode:** Upon trigger, the ASR-PRO autonomously orchestrates an ambient response—activating breathing-light patterns, playing calming audio, and adjusting servos.

## 3. Hardware & Wiring

### Main ESP32 (Emogotchi Device)

| Component | Pin on ESP32 |
| :--- | :--- |
| **I2C Bus** | **`SCL -> GPIO 22`**, **`SDA -> GPIO 21`** |
| MPU6050 (GY-521) | SCL, SDA, `INT -> GPIO 4` |
| OLED (0.66" 64x48) | SCL, SDA (Address: 0x3C) |
| **Sensors** | |
| DHT11/22 | `Data -> GPIO 25` |
| Pulse Rate Sensor | `Signal -> GPIO 32` |
| Sound Sensor | `AO -> GPIO 34` |
| LDR (Light Sensor) | `AO -> GPIO 33` |
| **Actuators** | |
| KY-009 RGB LED | `R -> GPIO 12`, `G -> GPIO 13`, `B -> GPIO 14`, `GND -> GND` |
| Heartbeat LED | `LED -> GPIO 2` |
| **Power** | `VCC -> 3.3V`, `GND -> GND` |

### Second ESP32 (Smart Home Hub)

| Category | Component | Pin(s) on ESP32 |
| :--- | :--- | :--- |
| **I2C Bus** | SCL | GPIO 22 |
| | SDA | GPIO 21 |
| | OLED (0.66" 64x48) | Uses I2C Bus (Address: 0x3C) |
| **Actuators** | Servo (Door) | GPIO 15 |
| | Servo (Window) | GPIO 16 |
| | RGB LED | R: GPIO 12, G: GPIO 13, B: GPIO 14 |
| **Status Pins** | Stress0 | GPIO 18 |
| | Stress1 | GPIO 17 |
| **Inputs** | Door Button | GPIO 25 |
| | Window Button | GPIO 26 |
| | RGB Button | GPIO 27 |
| | Relax Button | GPIO 32 |
| **Power** | VCC | 5V (VIN) |
| | GND | GND |

---

## 4. Software Setup

### A. Cloud Server (DigitalOcean Droplet)

1.  **Create Droplet:**
    * **Image:** Ubuntu 22.04 (LTS)
    * **Plan:** Basic Regular
    * **Region:** Singapore
    * **Authentication:** Password

2.  **SSH into Server:**
    ```bash
    ssh root@[your_droplet_ip]
    ```

3.  **Install System Dependencies:**
    ```bash
    apt update
    apt install mosquitto mosquitto-clients git python3-pip python3-venv
    ```

4.  **Configure Mosquitto (Broker):**
    * Edit config: `nano /etc/mosquitto/conf.d/default.conf`
    * Add:
        ```
        listener 1883 0.0.0.0
        allow_anonymous true
        ```
    * Restart: `systemctl restart mosquitto`

5.  **Configure Firewall:**
    ```bash
    ufw allow ssh
    ufw allow 1883
    ufw allow 5000
    ufw enable
    ```

6.  **Set Up Project Code (Flask Server):**
    ```bash
    cd ~
    git clone https://github.com/CPLADRAGON/CS3237-Emogotchi.git
    cd CS3237-Emogotchi
    
    # Create and activate virtual environment
    python3 -m venv venv
    source venv/bin/activate
    
    # Install Python libraries (Includes new Google AI dependency)
    pip install flask paho-mqtt pandas scikit-learn joblib numpy tensorflow requests google-generativeai
    
    # Train Model
    python train_model.py 
    ```

7.  **API Key Configuration:**
    * Ensure your `server.py` contains a valid `GOOGLE_API_KEY`.
    * Ensure the `ai_model` in `server.py` is set to `gemini-2.0-flash`.

### B. ESP32 Devices (Local)

1.  **IDE:** Arduino IDE with "ESP32" board package installed.
2.  **Libraries:** `PubSubClient`, `Adafruit MPU6050`, `Adafruit GFX`, `Adafruit SSD1306`, `DHT sensor library`, `ArduinoJson`, `ESP32Servo`.
3.  **Configure:** Update Wi-Fi credentials and MQTT Server IP in the `.ino` files.

### C. Telegram Bot Setup

1.  Create a bot via **BotFather** on Telegram.
2.  Get **Bot Token** and **Chat ID**.
3.  Update `BOT_TOKEN` and `CHAT_ID` in `server.py`.

---

## 5. How to Run

1.  **Start the Server:**
    * SSH into Droplet.
    * `cd CS3237-Emogotchi`
    * `source venv/bin/activate`
    * `python server.py`

2.  **Power On Devices:**
    * Power on ESP32s. Check Serial Monitor (115200 baud) for connection status.

3.  **View the Web UI:**
    * Go to **`http://[your_droplet_ip]:5000`**
    * **New:** Click the **"Stress / Advice"** card to view the AI-generated wellness tip in a popup modal.
    * **New:** Use the **24h / 7 Days** buttons on the chart to toggle history views.