from flask import Flask, request, jsonify, render_template, send_from_directory
import json
from collections import deque
import joblib
import pandas as pd
import time
import paho.mqtt.client as mqtt  # Used for both subscribing and publishing
import csv  # <-- ADDED for CSV logging
import os   # <-- ADDED for CSV logging
import threading  # <-- ADDED for thread-safe file writing

# --- Flask, Data Storage, Model ---
app = Flask(__name__)
MAX_HISTORY = 10
sensor_data_history = deque(maxlen=MAX_HISTORY)
latest_data = {}
latest_prediction = "N/A"
MODEL_PATH = 'stress_model.pkl'

# --- CSV Logging Setup ---
CSV_FILE_PATH = 'sensor_data.csv'
# Headers for the CSV file, matching Arduino keys + timestamp
CSV_HEADERS = ['timestamp', 'bpm', 'temperature',
               'humidity', 'noise', 'ldr', 'in_motion']
csv_lock = threading.Lock()  # To prevent race conditions when writing to file

# ---
# !!! IMPORTANT !!!
# Your model MUST be retrained with these exact features.
# The existing 'stress_model.pkl' will not work and will cause errors
# or give bad predictions.
# ---
feature_cols_expected_by_model = [
    # <-- ADDED ldr and in_motion
    'bpm', 'temperature', 'humidity', 'noise_db', 'ldr', 'in_motion'
]
model = None
try:
    model = joblib.load(MODEL_PATH)
    print(f"Machine learning model loaded from {MODEL_PATH}")
except Exception as e:
    print(f"Error loading model: {e}")

# --- MQTT Configuration ---
MQTT_BROKER_HOST = "localhost"  # Runs on the same machine as Flask
MQTT_BROKER_PORT = 1883
# --- MODIFIED: Matched topic to Arduino ---
MQTT_DATA_TOPIC = "esp32/sensor_data"  # <<< Topic to LISTEN on
MQTT_COMMAND_TOPIC = "emogotchi/window/command"  # <<< Topic to SEND on
STRESS_LEVEL_TO_TRIGGER = "Sad"  # Or "Stressed"

# --- This function will be called when we get a message ---


def on_message(client, userdata, msg):
    global latest_data, latest_prediction

    print(f"Received message on topic {msg.topic}: {msg.payload.decode()}")

    try:
        # 1. Decode and load the JSON data
        data = json.loads(msg.payload.decode())

        # --- Store the data for the UI ---
        data['timestamp'] = time.strftime('%H:%M:%S')
        latest_data = data
        sensor_data_history.append(data)

        # --- 4. Store data in CSV (Thread-safe) ---
        try:
            with csv_lock:
                # Extract only the data relevant to CSV headers
                row_data = {key: data.get(key) for key in CSV_HEADERS}

                with open(CSV_FILE_PATH, 'a', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=CSV_HEADERS)
                    # Header is written by init_csv(), not here
                    writer.writerow(row_data)
        except Exception as e:
            print(f"Error writing to CSV: {e}")

        # --- 2. Run Machine Learning Prediction ---
        prediction_result = "N/A"
        if model:
            try:
                # Map incoming data to model features
                input_data_mapped = {}
                input_data_mapped['bpm'] = data.get('bpm', 0)
                input_data_mapped['temperature'] = data.get('temperature', 0)
                input_data_mapped['humidity'] = data.get('humidity', 0)
                # Maps "noise" from Arduino to "noise_db"
                input_data_mapped['noise_db'] = data.get('noise', 0)
                input_data_mapped['ldr'] = data.get('ldr', 0)  # <-- ADDED
                input_data_mapped['in_motion'] = data.get(
                    'in_motion', 0)  # <-- ADDED

                # ... (Fill missing/default values as in your previous code) ...
                for col in feature_cols_expected_by_model:
                    if col not in input_data_mapped:
                        input_data_mapped[col] = 0

                input_df = pd.DataFrame([input_data_mapped])[
                    feature_cols_expected_by_model]
                prediction = model.predict(input_df)
                prediction_result = prediction[0]
                latest_prediction = prediction_result
                print(f"Prediction: {prediction_result}")

                # --- 3. Publish a command if "Stressed" ---
                if prediction_result == STRESS_LEVEL_TO_TRIGGER:
                    command = "OPEN"
                    mqtt_client.publish(MQTT_COMMAND_TOPIC, command)
                    print(
                        f"Published command '{command}' to topic '{MQTT_COMMAND_TOPIC}'")

            except Exception as e:
                print(f"Error during prediction: {e}")
                latest_prediction = "Prediction Error"
        else:
            print("Model not loaded, skipping prediction.")
            latest_prediction = "N/A - Model Error"

    except json.JSONDecodeError:
        print("Error: Received invalid JSON from ESP32")
    except Exception as e:
        print(f"An error occurred in on_message: {e}")


# --- MQTT Client Setup ---
mqtt_client = mqtt.Client(client_id="flask_server")
mqtt_client.on_message = on_message  # Attach the callback function


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        # --- Subscribe to the data topic ---
        client.subscribe(MQTT_DATA_TOPIC)
        print(f"Subscribed to topic: {MQTT_DATA_TOPIC}")
    else:
        print(f"Failed to connect to MQTT Broker, return code {rc}")


mqtt_client.on_connect = on_connect
try:
    mqtt_client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
    mqtt_client.loop_start()  # Start network loop
    print("MQTT Client configured.")
except Exception as e:
    print(f"Could not connect to MQTT Broker: {e}")
    mqtt_client = None

# --- Flask Routes for the Web UI (Unchanged) ---


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/data')
def get_data():
    return jsonify({
        "latest": latest_data,
        "history": list(sensor_data_history),
        "prediction": latest_prediction
    })

# --- This route is no longer used by the ESP32 ---


@app.route('/api/upload', methods=['POST'])
def http_upload():
    print("Received an HTTP POST request. This route is deprecated.")
    return jsonify({"status": "deprecated", "message": "Please use MQTT"}), 404

# --- ADDED: Function to initialize CSV file ---


def init_csv():
    with csv_lock:
        if not os.path.exists(CSV_FILE_PATH):
            try:
                with open(CSV_FILE_PATH, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=CSV_HEADERS)
                    writer.writeheader()
                print(f"Created new CSV file: {CSV_FILE_PATH}")
            except Exception as e:
                print(f"Error creating CSV file: {e}")


@app.route('/download')
def download_csv():
    try:
        # Assumes server.py is in the same directory as sensor_data.csv
        # If not, adjust the directory path.
        return send_from_directory(
            directory='.',
            path='sensor_data.csv',
            as_attachment=True
        )
    except FileNotFoundError:
        return "Error: File not found.", 404


# --- Run Flask ---
if __name__ == '__main__':
    init_csv()  # <-- ADDED: Check and create CSV on startup
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
