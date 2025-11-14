# Emogotchi - CS3237 IoT Project (Team 06)

**Course:** CS3237 Introduction to Internet of Things (AY2025/26 Semester 1)
**Team Members:** DONG HUAISHUO, LI ZHUOLUN, WANG BOYU, ZHANG HAOYU

---

## 1. Project Description

Students often spend long hours studying and can become unaware of accumulating environmental and physiological stress. This lack of awareness can lead to reduced productivity and long-term health issues.

**Emogotchi** is a real-time monitoring system designed to help students understand their well-being. The system uses a multi-sensor ESP32 device to capture physiological data (BPM, motion) and environmental data (temperature, humidity, noise, light). This data is published via MQTT to a cloud server, which uses an **LSTM (Long Short-Term Memory) machine learning model** to predict a "happiness score" based on the last 10 data points.

This prediction is then sent back to the ESP32, providing a **closed-loop feedback system**. The device displays the user's current stress state (e.g., "Happy", "Normal", "Sad") on an OLED screen and a dynamic RGB LED. The server also logs all data, provides a live web dashboard, and can send critical alerts via **Telegram**.

## 2. System Architecture

The project is built on a high-performance, real-time MQTT architecture.

1.  **Main ESP32 (Publisher/Subscriber):**
    * Runs **FreeRTOS** to manage 10+ concurrent tasks (sensors, display, network) across both CPU cores.
    * **Core 1 (Sensors):** Gathers data from the MPU6050 (interrupt-driven), DHT11, Pulse Sensor, Sound Sensor, and LDR.
    * **Core 0 (Network):** Manages Wi-Fi and MQTT.
    * **Publishes** a JSON payload of averaged sensor data to the `esp32/sensor_data` topic every 30 seconds.
    * **Subscribes** to the `esp32/prediction` topic to receive real-time commands from the server.

2.  **Cloud Server (DigitalOcean Droplet):**
    * Runs the **Mosquitto MQTT Broker** as the central message hub.
    * Runs a **Python Flask Server** (`server.py`) as the system's "brain":
        * **Subscribes** to `esp32/sensor_data`.
        * Collects a sequence of 10 data points.
        * Feeds the sequence into a trained **Keras/TensorFlow LSTM model** to predict a "happiness score" (0-100).
        * Maps the score to a state ("Happy", "Normal", "Sad").
        * **Publishes** the state and score (e.g., `"Sad:25.0"`) back to the `esp32/prediction` topic.
        * Sends a **Telegram alert** if the score drops below a critical threshold.
        * Logs all sensor data to `sensor_data.csv` and trend data to `happiness_trend.csv`.
        * **Serves a web UI** at `http://<server_ip>:5000` for live data visualization.

3.  **ESP32 Actuators (Feedback Loop):**
    * The **`mqttCallback`** function on the main ESP32 receives the prediction.
    * `Task_OLED` updates the 64x48 OLED screen with the current emotion and score.
    * `Task_LED_Control` updates the KY-009 RGB LED with a dynamic effect (e.g., breathing green for "Happy", solid blue for "Normal", blinking red for "Sad").

![A diagram showing the data flow from the ESP32 to the MQTT broker, to the Flask server, back to the broker, and finally to the display.](https://i.imgur.com/gA9mZ3e.png)

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