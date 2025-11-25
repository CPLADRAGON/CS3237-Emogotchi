# Emogotchi - CS3237 IoT Project (Team 06)

**Course:** CS3237 Introduction to Internet of Things (AY2025/26 Semester 1)  
**Team Members:** DONG HUAISHUO, LI ZHUOLUN, WANG BOYU, ZHANG HAOYU

---

## 1. Project Description

Students often spend long hours studying and can become unaware of accumulating environmental and physiological stress. This lack of awareness can lead to reduced productivity and long-term health issues.

**Emogotchi** is a real-time, multi-device IoT system designed to monitor a student's well-being. The system uses a multi-sensor **Main ESP32** device to capture physiological data (BPM, motion) and environmental data (temperature, humidity, noise, light).

This data is published via MQTT to a cloud server, which uses a time-series **LSTM (Long Short-Term Memory) machine learning model** to predict a "happiness score" (0-100).

**New: AI Wellness Coach & Dashboard V2.0**  
The system now integrates **Google Gemini 2.0 Flash AI**. It analyzes sensor data to generate context-aware wellness tips. The web interface has been completely overhauled into a **"Digital Twin" Dashboard**, featuring an animated **Emo-Avatar** that visually mimics the user's emotion (smiling, neutral, or stressed) in real-time, along with live connection monitoring and animated gauges.

This creates a **closed-loop feedback system**:
1.  **Emogotchi Device:** Displays current emotion ("Happy", "Normal", "Sad") on OLED and RGB LED.
2.  **Smart Home Hub:** Triggers physical actuators (servos, lights) and "Relax Mode" routines.
3.  **Web Dashboard:** Serves as the central emotional monitor with live visual feedback and AI coaching.
4.  **Telegram:** Sends critical alerts if the score drops below a threshold.

## 2. System Architecture

The project is built on a high-performance, real-time **Publish/Subscribe (Pub/Sub) model** using an MQTT broker.

1.  **Main ESP32 (Emogotchi Device):**
    * **Publisher/Subscriber**
    * Runs **FreeRTOS** to manage 10 concurrent tasks (sensors, display, network) across both CPU cores.
    * **Core 1 (Sensors):** Gathers data from the MPU6050 (interrupt-driven), DHT11, Pulse Sensor, Sound Sensor, and LDR.
    * **Core 0 (Network):** Manages Wi-Fi and MQTT.
    * **Publishes** JSON sensor data to `esp32/sensor_data`.
    * **Subscribes** to `esp32/prediction` to receive real-time commands.
    * Displays the received emotion on its OLED screen and RGB LED.

2.  **Cloud Server (DigitalOcean Droplet):**
    * **Broker & Brain**
    * Runs the **Mosquitto MQTT Broker** as the central message hub.
    * Runs a **Python Flask Server** (`server.py`):
        * **Subscribes** to `esp32/sensor_data`.
        * Feeds data into a trained **Keras/TensorFlow LSTM model** to predict happiness (0-100).
        * **Generative AI:** Calls **Google Gemini 2.0 Flash** to generate wellness advice.
        * **Publishes** emotion/score back to `esp32/prediction`.
        * Sends **Telegram alerts** for critical stress.
        * **Serves Dashboard V2.0:**
            * **Emo-Avatar:** An animated CSS face that reacts to the predicted emotion.
            * **Live Pulse:** "Heartbeat" monitor that detects if the device goes offline.
            * **Interactive Stress Card:** Popup modal with AI-generated advice.
            * **Visuals:** Circular animated gauges, glassmorphism UI, and ambient background color shifts.
            * **Trends:** Toggle between 24-Hour and 7-Day history charts.

3.  **Smart Home Hub (ASR-PRO & Second ESP32):**
    * **Voice & Emotion Actuator Controller**
    * **Subscribes** to the `esp32/prediction` topic.
    * **ASR-PRO Module:** Primary offline voice recognition engine and actuator controller.
    * **Sentiment Gateway (Second ESP32):** Bridges cloud data to the offline hub. Triggers "Relax Mode" via interrupt when stress is detected.
    * **Relax Mode:** Autonomously orchestrates ambient responses (breathing lights, calming audio, servo movement).

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
    
    # Install Python libraries
    # Includes google-generativeai for AI and python-dotenv for security
    pip install flask paho-mqtt pandas scikit-learn joblib numpy tensorflow requests google-generativeai python-dotenv
    
    # Train Model
    python train_model.py 
    ```

7.  **Secure Configuration (.env):**
    *   **Do not hardcode API keys.** Create a `.env` file in the project directory:
        ```bash
        nano .env
        ```
    *   Add your keys inside:
        ```ini
        GOOGLE_API_KEY=your_gemini_api_key_here
        TELEGRAM_BOT_TOKEN=your_telegram_bot_token_here
        TELEGRAM_CHAT_ID=your_chat_id_here
        ```
    *   Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

### B. ESP32 Devices (Local)

1.  **IDE:** Arduino IDE with "ESP32" board package installed.
2.  **Libraries:** `PubSubClient`, `Adafruit MPU6050`, `Adafruit GFX`, `Adafruit SSD1306`, `DHT sensor library`, `ArduinoJson`, `ESP32Servo`.
3.  **Configure:** Update Wi-Fi credentials and MQTT Server IP in the `.ino` files.

### C. Telegram Bot Setup

1.  Create a bot via **BotFather** on Telegram.
2.  Get **Bot Token** and **Chat ID**.
3.  Add them to your `.env` file as shown above.

---

## 5. How to Run

1.  **Start the Server:**
    * SSH into Droplet.
    * `cd CS3237-Emogotchi`
    * `source venv/bin/activate`
    * `python server.py`

2.  **Power On Devices:**
    * Power on ESP32s. Check Serial Monitor (115200 baud) for connection status.

3.  **View the Dashboard V2.0:**
    * Go to **`http://[your_droplet_ip]:5000`**
    * Observe the **Emo-Avatar** changing expression based on real-time data.
    * Check the **Live Status Pulse** (Green = Online, Red = Offline).
    * Click the **"AI Coach"** card to view personalized advice.