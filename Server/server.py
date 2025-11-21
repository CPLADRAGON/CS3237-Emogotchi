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
import numpy as np
from tensorflow.keras.models import load_model
from datetime import datetime, timedelta, timezone
import requests

# --- Flask, Data Storage ---
app = Flask(__name__)
TIME_STEPS = 10
MAX_HISTORY = TIME_STEPS
sensor_data_history = deque(maxlen=MAX_HISTORY)
latest_data = {}
latest_prediction = "Waiting for data..."
latest_happiness_score = 0.0

# --- Model Paths ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'emogotchi_lstm_regressor.h5')
SCALER_PATH = os.path.join(SCRIPT_DIR, 'sensor_scaler.pkl')
TARGET_SCALER_PATH = os.path.join(SCRIPT_DIR, 'target_scaler.pkl')

features = ['bpm', 'temperature', 'humidity', 'noise', 'ldr', 'in_motion']

try:
    model = load_model(MODEL_PATH, compile=False)
    feature_scaler = joblib.load(SCALER_PATH)
    target_scaler = joblib.load(TARGET_SCALER_PATH)
    print(f"Successfully loaded LSTM model, feature scaler, and target scaler.")
except Exception as e:
    print(f"FATAL ERROR: Could not load models: {e}")
    model = None

# --- CSV Logging Setup ---
CSV_FILE_PATH = 'sensor_data.csv'
CSV_HEADERS = ['timestamp', 'bpm', 'temperature',
               'humidity', 'noise', 'ldr', 'in_motion', 'happiness_score']

TREND_CSV_FILE_PATH = os.path.join(SCRIPT_DIR, 'happiness_trend.csv')
TREND_CSV_HEADERS = ['timestamp', 'happiness_score']
csv_lock = threading.Lock()

# --- [ ADDED: Telegram Notification Setup ] ---
BOT_TOKEN = "7393315205:AAEos38jymwEA4lhCUZQBWfZY8U5ZxdwlqY"
CHAT_ID = "-5025276308"  # Must be a string, e.g., "-100123456789"
NOTIFICATION_COOLDOWN_SEC = 10
g_last_notification_time = 0.0  # Global variable to track cooldown

# --- MQTT Configuration ---
MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
MQTT_DATA_TOPIC = "esp32/sensor_data"
MQTT_COMMAND_TOPIC = "esp32/prediction"

# --- Helper Functions (on_connect, get_happiness_score, map_score_to_emotion) ---


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(MQTT_DATA_TOPIC)
        print(f"Subscribed to topic: {MQTT_DATA_TOPIC}")
    else:
        print(f"Failed to connect to MQTT Broker, return code {rc}")

# --- [ ADDED: Telegram Helper Function ] ---


def send_telegram_notification(message):
    """Sends a message to Telegram in a non-blocking thread."""
    def send_message_in_thread():
        try:
            url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage"
            payload = {"chat_id": CHAT_ID, "text": message}
            response = requests.post(
                url, data=payload, timeout=5)  # 5 sec timeout
            if response.status_code == 200:
                print("Telegram notification sent successfully.")
            else:
                print(
                    f"Error sending Telegram notification: {response.status_code} - {response.text}")
        except Exception as e:
            print(f"Exception while sending Telegram notification: {e}")

    # Start the send in a new thread to avoid blocking the server
    thread = threading.Thread(target=send_message_in_thread)
    thread.start()


def get_happiness_score(sensor_sequence):
    global model, feature_scaler, target_scaler, features
    if not model:
        return 50.0
    emotion_score = -1
    latest_reading = sensor_sequence[-1]

    # Hard thresholds
    if latest_reading['bpm'] > 130 and latest_reading['in_motion'] == 0:
        emotion_score = 10.0
    elif latest_reading['temperature'] > 36:
        emotion_score = 15.0

    # LSTM Model
    if emotion_score == -1:
        try:
            # Use .get(f, 0) to prevent errors if a key is missing
            data_list = [[reading.get(f, 0) for f in features]
                         for reading in sensor_sequence]
            data_array = np.array(data_list)
            data_scaled = feature_scaler.transform(data_array)
            data_lstm = np.expand_dims(data_scaled, axis=0)
            scaled_score = model.predict(data_lstm, verbose=0)[0]
            emotion_score = target_scaler.inverse_transform(
                scaled_score.reshape(-1, 1))[0][0]

            # --- [ THIS IS THE CRITICAL FIX ] ---
            # Check for NaN *before* clipping
            if np.isnan(emotion_score):
                print("Warning: Model returned NaN. Defaulting to 50.")
                emotion_score = 50.0
            # --- [ END OF FIX ] ---

            emotion_score = np.clip(emotion_score, 0, 100)

        except Exception as e:
            print(f"Error during LSTM prediction: {e}")
            emotion_score = 50.0
    return round(emotion_score, 1)


def map_score_to_emotion(score):
    if score < 34:
        return "Sad"
    elif score < 67:
        return "Normal"
    else:
        return "Happy"

# --- Main MQTT Callback ---


def on_message(client, userdata, msg):
    global latest_data, latest_prediction, latest_happiness_score, sensor_data_history

    try:
        data = json.loads(msg.payload.decode())

        # --- [ MODIFIED: Use timezone-aware ISO format ] ---
        data['timestamp'] = datetime.now(timezone.utc).isoformat()
        latest_data = data  # Store for /data endpoint

        # --- Prediction Logic ---
        sensor_data_history.append(data)  # Add data to the 10-item deque

        if len(sensor_data_history) < TIME_STEPS:
            # Not enough data yet
            print(
                f"Gathering data... {len(sensor_data_history)}/{TIME_STEPS} samples.")
            latest_prediction = "Waiting for data..."
            latest_happiness_score = 0.0

        else:
            # --- We have 10 samples, run the prediction ---
            sequence = list(sensor_data_history)
            score = get_happiness_score(sequence)

            # --- [ ADDED: Telegram Alert Logic ] ---
            global g_last_notification_time
            if score < 33:
                current_time = time.time()
                # Check if cooldown has passed
                if (current_time - g_last_notification_time) > NOTIFICATION_COOLDOWN_SEC:
                    print(f"Score is {score}, triggering Telegram alert.")
                    g_last_notification_time = current_time  # Reset cooldown
                    message = f"🚨 LOW HAPPINESS ALERT 🚨\n\nDevice '{data.get('device_name', 'ESP32')}' reports a happiness score of: {score}"
                    send_telegram_notification(message)
                else:
                    print(f"Score is {score}, but in notification cooldown.")

            # Update global state
            latest_happiness_score = score
            prediction_result = map_score_to_emotion(score)
            latest_prediction = prediction_result

            # Add the score to the 'data' dict *before* logging
            data['happiness_score'] = score

            # Publish command to ESP32
            command = f"{prediction_result}:{score}"
            mqtt_client.publish(MQTT_COMMAND_TOPIC, command)
            print(f"Prediction: {command}")

            # --- [ LOGGING BLOCK ] ---
            try:
                with csv_lock:
                    # Log to main sensor_data.csv
                    with open(CSV_FILE_PATH, 'a', newline='') as f:
                        writer = csv.DictWriter(f, fieldnames=CSV_HEADERS)
                        writer.writerow({key: data.get(key)
                                        for key in CSV_HEADERS})

                    # Log to trend.csv
                    trend_row = {
                        'timestamp': data['timestamp'], 'happiness_score': score}
                    with open(TREND_CSV_FILE_PATH, 'a', newline='') as f:
                        writer = csv.DictWriter(
                            f, fieldnames=TREND_CSV_HEADERS)
                        writer.writerow(trend_row)
            except Exception as e:
                print(f"Error writing to CSV: {e}")
            # --- [ END OF LOGGING BLOCK ] ---

    except Exception as e:
        print(f"An error occurred in on_message: {e}")


# --- MQTT Client Setup ---
mqtt_client = mqtt.Client(client_id="flask_server")
mqtt_client.on_message = on_message
mqtt_client.on_connect = on_connect
try:
    mqtt_client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
    mqtt_client.loop_start()
    print("MQTT Client configured.")
except Exception as e:
    print(f"Could not connect to MQTT Broker: {e}")
    mqtt_client = None

# --- Flask Routes ---


@app.route('/')
def index():
    return render_template('index.html')

# --- [ ADDED: Test Endpoint for Telegram ] ---


@app.route('/info')
def info():
    return render_template('info.html')


@app.route('/test_alert')
def test_alert():
    """Manually triggers a test notification."""
    print("--- MANUAL ALERT TEST TRIGGERED ---")
    message = "🔔 This is a test notification from the Emogotchi server. If you see this, the bot is working!"
    send_telegram_notification(message)
    return "Test alert sent! Check your Telegram group."


@app.route('/data')
def get_data():
    return jsonify({
        "latest": latest_data,
        "history": list(sensor_data_history),
        "prediction": latest_prediction,
        "happiness_score": latest_happiness_score
    })

# --- [ MODIFIED: Trend Endpoint now calculates hourly average ] ---


@app.route('/trend_data')
def get_trend_data():
    try:
        # 1. Read the trend data
        df = pd.read_csv(TREND_CSV_FILE_PATH)

        # 2. Convert timestamp column to datetime objects
        # --- [ THIS IS THE ROBUST FIX ] ---
        # errors='coerce' turns bad data into NaT
        # utc=True forces all parsed timestamps into the UTC timezone
        df['timestamp'] = pd.to_datetime(
            df['timestamp'], errors='coerce', utc=True)
        # --- [ END OF FIX ] ---

        # 3. Drop any rows where the timestamp was bad
        df.dropna(subset=['timestamp'], inplace=True)

        # 4. Get the cutoff for 24 hours ago (which is already UTC)
        one_day_ago = datetime.now(timezone.utc) - timedelta(hours=24)

        # 5. Filter the DataFrame for the last 24 hours
        # This comparison will now work
        df_filtered = df[df['timestamp'] >= one_day_ago]

        # 6. Replace any NaN/NaT with None, which becomes 'null' in JSON
        df_final = df_filtered.where(pd.notnull(df_filtered), None)

        # 7. Convert back to ISO string for JSON
        df_final['timestamp'] = df_final['timestamp'].apply(
            lambda x: x.isoformat() if pd.notnull(x) else None)

        # 8. Return the raw, filtered, and CLEANED data as JSON
        return jsonify(df_final.to_dict(orient='records'))

    except FileNotFoundError:
        return jsonify([])  # Send empty list if file doesn't exist yet
    except Exception as e:
        print(f"Error in /trend_data: {e}")
        return jsonify({"error": str(e)}), 500
# --- CSV Init Functions ---


def init_csv(path, headers):
    with csv_lock:
        if not os.path.exists(path):
            try:
                with open(path, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=headers)
                    writer.writeheader()
                print(f"Created new CSV file: {path}")
            except Exception as e:
                print(f"Error creating CSV file {path}: {e}")

# ... (other routes like /download) ...


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
    # Init both CSV files
    init_csv(CSV_FILE_PATH, CSV_HEADERS)
    init_csv(TREND_CSV_FILE_PATH, TREND_CSV_HEADERS)
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
