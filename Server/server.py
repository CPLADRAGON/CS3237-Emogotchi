from flask import Flask, request, jsonify, render_template, send_from_directory
import json
from collections import deque
import joblib
import pandas as pd
import time
import paho.mqtt.client as mqtt
import csv
import os
import threading

# --- [ ADDED: New Imports for LSTM Model ] ---
import numpy as np
from tensorflow.keras.models import load_model

# --- Flask, Data Storage ---
app = Flask(__name__)
TIME_STEPS = 10  # <-- ADDED: Must match the model's training
MAX_HISTORY = TIME_STEPS
# This deque is now our LSTM input sequence
sensor_data_history = deque(maxlen=MAX_HISTORY)
latest_data = {}
latest_prediction = "Waiting for data..."  # <-- MODIFIED: New default message
latest_happiness_score = 0.0  # <-- ADDED: To store the 0-100 score

# --- [ MODIFIED: Load New LSTM Model and Scalers ] ---
MODEL_PATH = 'emogotchi_lstm_regressor.keras'
SCALER_PATH = 'sensor_scaler.pkl'
TARGET_SCALER_PATH = 'target_scaler.pkl'

# These are the 6 features the model was trained on, in order.
features = ['bpm', 'temperature', 'humidity', 'noise', 'ldr', 'in_motion']

try:
    model = load_model(MODEL_PATH)
    feature_scaler = joblib.load(SCALER_PATH)
    target_scaler = joblib.load(TARGET_SCALER_PATH)
    print(f"Successfully loaded LSTM model, feature scaler, and target scaler.")
except Exception as e:
    print(f"FATAL ERROR: Could not load models: {e}")
    model = None

# --- CSV Logging Setup ---
CSV_FILE_PATH = 'sensor_data.csv'
CSV_HEADERS = ['timestamp', 'bpm', 'temperature',
               'humidity', 'noise', 'ldr', 'in_motion']
csv_lock = threading.Lock()

# --- MQTT Configuration ---
MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
MQTT_DATA_TOPIC = "esp32/sensor_data"
MQTT_COMMAND_TOPIC = "esp32/prediction"

# --- [ ADDED: Prediction functions from your notebook ] ---


def get_happiness_score(sensor_sequence):
    """
    Takes a sequence of 10 sensor data dictionaries, checks hard thresholds,
    runs the LSTM model, and returns only the 0-100 score.

    :param sensor_sequence: A list of 10 dictionaries, from oldest to newest.
    :return: A float (0-100) representing the happiness score.
    """
    global model, feature_scaler, target_scaler, features

    if not model:
        print("Model not loaded, returning default score.")
        return 50.0

    emotion_score = -1

    # Get the MOST RECENT sensor reading for hard thresholds
    latest_reading = sensor_sequence[-1]

    # --- 1. Check Hard Thresholds First ---
    if latest_reading['bpm'] > 130 and latest_reading['in_motion'] == 0:
        emotion_score = 10.0
    elif latest_reading['temperature'] > 32:
        emotion_score = 15.0

    # --- 2. If no hard rule, use LSTM Model ---
    if emotion_score == -1:
        try:
            # Convert list of dicts to 2D numpy array based on 'features' list
            data_list = [[reading[f] for f in features]
                         for reading in sensor_sequence]
            data_array = np.array(data_list)

            # Scale features
            data_scaled = feature_scaler.transform(data_array)

            # Reshape for LSTM: (1 sample, 10 time steps, 6 features)
            data_lstm = np.expand_dims(data_scaled, axis=0)

            # Predict the *scaled* emotion score
            scaled_score = model.predict(data_lstm, verbose=0)[0]

            # Inverse-transform the score to 0-100
            emotion_score = target_scaler.inverse_transform(
                scaled_score.reshape(-1, 1))[0][0]

            # Clip score to be safe (0-100)
            emotion_score = np.clip(emotion_score, 0, 100)

        except Exception as e:
            print(f"Error during LSTM prediction: {e}")
            emotion_score = 50.0

    # --- 3. Return only the numeric score ---
    return round(emotion_score, 1)


def map_score_to_emotion(score):
    """Converts the 0-100 happiness score to a string."""
    if score < 34:
        return "Sad"
    elif score < 67:
        return "Normal"
    else:
        return "Happy"

# --- This function will be called when we get a message ---


def on_message(client, userdata, msg):
    global latest_data, latest_prediction, latest_happiness_score, sensor_data_history

    print(f"Received message on topic {msg.topic}: {msg.payload.decode()}")

    try:
        # 1. Decode and load the JSON data
        data = json.loads(msg.payload.decode())

        # --- 2. Store the data ---
        data['timestamp'] = time.strftime('%H:%M:%S')
        latest_data = data
        sensor_data_history.append(data)  # The deque handles the history

        # --- 3. Store data in CSV (Thread-safe) ---
        try:
            with csv_lock:
                row_data = {key: data.get(key) for key in CSV_HEADERS}
                with open(CSV_FILE_PATH, 'a', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=CSV_HEADERS)
                    writer.writerow(row_data)
        except Exception as e:
            print(f"Error writing to CSV: {e}")

        # --- [ MODIFIED: New Prediction Logic ] ---
        # 4. Check if we have enough data to predict
        if len(sensor_data_history) < TIME_STEPS:
            print(
                f"Gathering data... {len(sensor_data_history)}/{TIME_STEPS} samples.")
            latest_prediction = "Waiting for data..."
            latest_happiness_score = 0.0

        else:
            # We have a full sequence (10 readings)
            print("Full sequence (10 samples) received. Running prediction...")

            # Convert deque to a simple list for the function
            sequence = list(sensor_data_history)

            # 5. Get 0-100 score from LSTM
            score = get_happiness_score(sequence)
            latest_happiness_score = score  # Save for dashboard

            # 6. Map score to emotion
            prediction_result = map_score_to_emotion(score)
            latest_prediction = prediction_result  # Save for dashboard

            print(f"Prediction: Score={score}, Emotion='{prediction_result}'")

            # 7. Publish the command
            command = prediction_result
            mqtt_client.publish(MQTT_COMMAND_TOPIC, command)
            print(
                f"Published prediction '{command}' to topic '{MQTT_COMMAND_TOPIC}'")

    except json.JSONDecodeError:
        print("Error: Received invalid JSON from ESP32")
    except Exception as e:
        print(f"An error occurred in on_message: {e}")


# --- MQTT Client Setup ---
mqtt_client = mqtt.Client(client_id="flask_server")
mqtt_client.on_message = on_message
mqtt_client.on_connect = on_connect  # We define on_connect below
try:
    mqtt_client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
    mqtt_client.loop_start()
    print("MQTT Client configured.")
except Exception as e:
    print(f"Could not connect to MQTT Broker: {e}")
    mqtt_client = None


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(MQTT_DATA_TOPIC)
        print(f"Subscribed to topic: {MQTT_DATA_TOPIC}")
    else:
        print(f"Failed to connect to MQTT Broker, return code {rc}")

# --- Flask Routes for the Web UI ---


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/data')
def get_data():
    # --- [ MODIFIED: Added happiness_score ] ---
    return jsonify({
        "latest": latest_data,
        "history": list(sensor_data_history),
        "prediction": latest_prediction,
        "happiness_score": latest_happiness_score  # Optional: for the dashboard
    })

# ... (rest of your Flask routes: /api/upload, init_csv, /download) ...


@app.route('/api/upload', methods=['POST'])
def http_upload():
    print("Received an HTTP POST request. This route is deprecated.")
    return jsonify({"status": "deprecated", "message": "Please use MQTT"}), 404


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
        return send_from_directory(
            directory='.',
            path='sensor_data.csv',
            as_attachment=True
        )
    except FileNotFoundError:
        return "Error: File not found.", 404


# --- Run Flask ---
if __name__ == '__main__':
    init_csv()
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
