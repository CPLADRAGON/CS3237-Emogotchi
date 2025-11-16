# Emogotchi - CS3237 IoT Project (Team 06)

**Course:** CS3237 Introduction to Internet of Things (AY2025/26 Semester 1)  
**Team Members:** DONG HUAISHUO, LI ZHUOLUN, WANG BOYU, ZHANG HAOYU

---

## 1. Project Description

Students often spend long hours studying and can become unaware of accumulating environmental and physiological stress. This lack of awareness can lead to reduced productivity and long-term health issues.

**Emogotchi** is a real-time, multi-device IoT system designed to monitor a student's well-being. The system uses a multi-sensor **Main ESP32** device to capture physiological data (BPM, motion) and environmental data (temperature, humidity, noise, light).

This data is published via MQTT to a cloud server, which uses a time-series **LSTM (Long Short-Term Memory) machine learning model** to predict a "happiness score" (0-100) based on the last 10 data points.

This prediction is then sent back to two subscriber devices, creating a **closed-loop feedback system**:
1.  The **Main ESP32 (Emogotchi)** displays the user's current emotion (e.g., "Happy", "Normal", "Sad") on its OLED screen and a dynamic RGB LED.
2.  A **Second ESP32 (Smart Home Hub)** receives the same data, displays an emoticon on its own OLED, and can trigger "relax mode" routines (e.g., mood lighting, or servos) to actively improve the user's environment.

The server also logs all data to CSV, provides a live web dashboard with 24-hour trend analysis, and can send critical alerts via **Telegram** if the user's happiness score is too low.

## 2. System Architecture

The project is built on a high-performance, real-time **Publish/Subscribe (Pub/Sub) model** using an MQTT broker.

1.  **Main ESP32 (Emogotchi Device):**
    * **Publisher/Subscriber**
    * Runs **FreeRTOS** to manage 10 concurrent tasks (sensors, display, network) across both CPU cores.
    * **Core 1 (Sensors):** Gathers data from the MPU6050 (interrupt-driven), DHT11, Pulse Sensor, Sound Sensor, and LDR.
    * **Core 0 (Network):** Manages Wi-Fi and MQTT.
    * **Publishes** a JSON payload of sensor data to the `esp32/sensor_data` topic.
    * **Subscribes** to the `esp32/prediction` topic to receive real-time commands from the server.
    * Displays the received emotion on its OLED screen and RGB LED.

2.  **Cloud Server (DigitalOcean Droplet):**
    * **Broker & Brain**
    * Runs the **Mosquitto MQTT Broker** as the central message hub.
    * Runs a **Python Flask Server** (`server.py`):
        * **Subscribes** to `esp32/sensor_data`.
        * Collects a sequence of 10 data points.
        * Feeds the sequence into a trained **Keras/TensorFlow LSTM model** (`emogotchi_lstm_regressor.h5`) to predict a "happiness score" (0-100).
        * Maps the score to an emotion ("Happy", "Normal", "Sad").
        * **Publishes** the emotion and score (e.g., `"Sad:22.5"`) back to the `esp32/prediction` topic.
        * Sends a **Telegram alert** if the score drops below a critical threshold.
        * Logs all sensor data to `sensor_data.csv` and trend data to `happiness_trend.csv`.
        * **Serves a web UI** at `http://<server_ip>:5000` for live data visualization.

3.  **Second ESP32 (Smart Home Hub):**
    * **Subscriber/Actuator**
    * **Subscribes** to the `esp32/prediction` topic.
    * Displays the received emotion as an emoticon on its own OLED.
    * Can be triggered (by a low score and a button press) into a "Relax Mode" to control actuators like servos (simulating opening a window) or mood lighting.
    * Also supports manual control of connected home devices (servos, LEDs) via local buttons.

### Data Flow Diagram

```mermaid
%% System Block Diagram for Emogotchi (CS3237 Team 06)
graph LR
    
    %% --- COLUMN 1: EDGE (SENSORS) ---
    subgraph 1. Edge (Data Collection)
        direction TB
        
        USER([<fa:fa-user> </fa:fa-user> User<br>(Physiology & Environment)])
        
        ESP_MAIN(
            [<fa:fa-microchip> </fa:fa-microchip> Emogotchi (ESP32-Main)]
            ---
            <b>Tasks (FreeRTOS Core 1):</b><br>
            - Task_HeartRate (ADC Mutex)<br>
            - Task_Sound (ADC Mutex)<br>
            - Task_LDR (ADC Mutex)<br>
            - Task_DHT<br>
            - Task_MPU (I2C Mutex)<br>
            - Task_Display
            <br>
            <b>Network (FreeRTOS Core 0):</b><br>
            - Task_MQTT_Loop<br>
            - Task_UploadData
        )
        
        SENSORS[<fa:fa-thermometer-half> </fa:fa-thermometer-half> Sensors:<br>- Pulse (BPM)<br>- DHT11 (Temp/Hum)<br>- Sound (Noise)<br>- LDR (Light)<br>- MPU6050 (Motion)]
        
        USER -- Sensed By --> SENSORS -- Raw Data --> ESP_MAIN
    end

    %% --- COLUMN 2: CLOUD (BROKER & SERVER) ---
    subgraph 2. Cloud (DigitalOcean Droplet)
        direction TB
        
        BROKER(<fa:fa-rss> </fa:fa-rss> Mosquitto MQTT Broker)
        
        SERVER(
            [<fa:fa-server> </fa:fa-server> Python Flask Server (server.py)]
            ---
            <b>Services:</b><br>
            - MQTT Subscriber (on_message)<br>
            - LSTM Model Prediction<br>
            - Data Logging (CSV)<br>
            - Telegram Alert Trigger<br>
            - Web Dashboard API (/data)<br>
            - MQTT Publisher (Commands)
        )
        
        ML_MODEL[<fa:fa-brain> </fa:fa-brain> LSTM Model<br>(emogotchi_lstm_regressor.h5)]
        STORAGE[<fa:fa-database> </fa:fa-database> Data Logs<br>(sensor_data.csv)<br>(happiness_trend.csv)]
        
        %% Cloud Data Flow
        BROKER -- 1. Sensor JSON --> SERVER
        SERVER -- 2. Collects 10 samples --> ML_MODEL
        ML_MODEL -- 3. Predicts Score (e.g., 22.5) --> SERVER
        SERVER -- 4. Logs Data --> STORAGE
    end
    
    %% --- COLUMN 3: OUTPUTS & FEEDBACK ---
    subgraph 3. Feedback & Actuation
        direction TB
        
        %% Main ESP32 Feedback
        ESP_MAIN_FB(
            [<fa:fa-eye> </fa:fa-eye> Emogotchi Feedback]
            ---
            - OLED Display (Emotion + Score)<br>
            - RGB LED (Breathing/Blinking)
        )
        
        %% Smart Home Hub
        ESP_SECOND(
            [<fa:fa-home> </fa:fa-home> Smart Home Hub (ESP32-Second)]
            ---
            - MQTT Subscriber<br>
            - Servo Control<br>
            - OLED (Emoticon)<br>
            - Local Button Input
        )
        
        %% Web & Telegram
        WEB_UI([<fa:fa-chart-line> </fa:fa-chart-line> Web UI (index.html)])
        TELEGRAM([<fa:fa-paper-plane> </fa:fa-paper-plane> Telegram Alert])
    end

    %% --- GLOBAL DATA FLOWS (ARROWS) ---
    
    %% ESP32 to Cloud
    ESP_MAIN -- <b>Publish:</b> esp32/sensor_data<br>(Sensor JSON) --> BROKER
    
    %% Cloud to Devices (Closed Loop)
    SERVER -- <b>Publish:</b> esp32/prediction<br>("Sad:22.5") --> BROKER
    BROKER -- <b>Subscribe:</b> esp32/prediction --> ESP_MAIN
    BROKER -- <b>Subscribe:</b> esp32/prediction --> ESP_SECOND
    
    %% Cloud to User
    SERVER -- <b>HTTP POST</b> (if Sad) --> TELEGRAM
    SERVER -- <b>HTTP GET /data</b> --> WEB_UI
    
    %% Final Outputs to User
    ESP_MAIN -- Displays --> ESP_MAIN_FB
    ESP_SECOND -- Triggers --> ESP_SECOND
    
    %% Style definitions
    classDef esp fill:#2E86C1,stroke:#FFF,color:#FFF,stroke-width:2px;
    classDef server fill:#27AE60,stroke:#FFF,color:#FFF,stroke-width:2px;
    classDef broker fill:#E67E22,stroke:#FFF,color:#FFF,stroke-width:2px;
    classDef user fill:#9B59B6,stroke:#FFF,color:#FFF,stroke-width:2px;
    classDef outputs fill:#414868,stroke:#c0caf5,color:#c0caf5,stroke-width:2px;

    class ESP_MAIN,ESP_SECOND esp;
    class SERVER,ML_MODEL,STORAGE server;
    class BROKER broker;
    class USER user;
    class WEB_UI,TELEGRAM,ESP_MAIN_FB outputs;

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
    * **Plan:** Basic Regular (cheapest option is fine)
    * **Region:** Singapore
    * **Authentication:** Password (create a root password)
    * Copy the Droplet's **public IP address**.

2.  **SSH into Server:**
    ```bash
    ssh root@[your_droplet_ip]
    ```

3.  **Install Software:**
    ```bash
    # Update package lists
    apt update
    
    # Install Mosquitto MQTT Broker and Git
    apt install mosquitto mosquitto-clients git
    
    # Install Python 3, pip, and venv (virtual environment)
    apt install python3-pip python3-venv
    ```

4.  **Configure Mosquitto (Broker):**
    * Create a new config file:
        ```bash
        nano /etc/mosquitto/conf.d/default.conf
        ```
    * Add these two lines to the file to allow all connections:
        ```
        listener 1883 0.0.0.0
        allow_anonymous true
        ```
    * Save (**Ctrl+O**) and Exit (**Ctrl+X**).
    * Restart Mosquitto to apply changes:
        ```bash
        systemctl restart mosquitto
        ```

5.  **Configure Firewall:**
    ```bash
    ufw allow ssh  # Port 22
    ufw allow 1883 # Port 1883 (MQTT)
    ufw allow 5000 # Port 5000 (Flask Web UI)
    ufw enable
    ```

6.  **Set Up Project Code (Flask Server):**
    ```bash
    # Go to your home directory
    cd ~
    
    # Clone your project repo
    git clone [https://github.com/CPLADRAGON/CS3237-Emogotchi.git](https://github.com/CPLADRAGON/CS3237-Emogotchi.git)
    cd CS3237-Emogotchi # Or your repo name
    
    # Create and activate a Python virtual environment
    python3 -m venv venv
    source venv/bin/activate
    
    # Install Python libraries
    # (You can also 'pip install -r requirements.txt' if you create one)
    pip install flask paho-mqtt pandas scikit-learn joblib numpy tensorflow requests
    
    # Run the ML training script to create the model/scaler files
    # (Make sure your training CSV is in the repo)
    python train_model.py 
    ```

### B. ESP32 Devices (Local)

1.  **IDE:** Use the Arduino IDE.
2.  **Boards Manager:** Install the **"ESP32"** board package.
3.  **Libraries:** Install the following from the Library Manager:
    * `PubSubClient` (by Nick O'Leary)
    * `Adafruit MPU6050`
    * `Adafruit GFX Library`
    * `Adafruit SSD1306`
    * `DHT sensor library`
    * `ArduinoJson`
    * `ESP32Servo` (for the optional 2nd ESP32)

4.  **Configure Code:**
    * Open the `.ino` file for your Main ESP32.
    * Update your Wi-Fi `ssid` and `password`.
    * Update `const char* mqtt_server` to your Droplet's public IP address.

### C. Telegram Bot Setup

1.  Open the Telegram app and search for the **BotFather**.
2.  Send `/newbot` and follow the instructions to create a bot.
3.  Copy the **Bot Token** (e.g., `7393315205:AAE...`) and paste it into `BOT_TOKEN` in `server.py`.
4.  Create a new Telegram **group** and add your new bot to it.
5.  Send a test message to the group (e.g., `/start`).
6.  Open your browser and go to this URL (replace with your token):
    `https://api.telegram.org/bot[YOUR_TOKEN]/getUpdates`
7.  Look for the `chat` object in the JSON response. Find the `id` (e.g., `-5025276308`) and paste it into `CHAT_ID` in `server.py`.

---

## 5. How to Run

1.  **Start the Server:**
    * SSH into your Droplet.
    * Navigate to your project folder: `cd CS3237-Emogotchi`
    * Activate the environment: `source venv/bin/activate`
    * Run the Flask application: `python server.py`
    * *(Optional: Use `screen -S flask` and then `python server.py` to keep it running after you disconnect).*

2.  **Power On Devices:**
    * Power on your Main ESP32.
    * Open the Arduino Serial Monitor (Baud: 115200) to watch its debug logs.

3.  **View the Web UI:**
    * Open a web browser on your computer or phone.
    * Go to your server's public address:
        **`http://[your_droplet_ip]:5000`**
    * You should see the dashboard populate with live data from your ESP32.