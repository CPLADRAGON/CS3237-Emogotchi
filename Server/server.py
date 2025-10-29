from flask import Flask, request, jsonify, render_template
import json
from collections import deque
import joblib  # To load the model
import pandas as pd  # To structure incoming data for the model
import time
# import paho.mqtt.client as mqtt # Optional: Uncomment if using MQTT commands

# Create a Flask web server application
app = Flask(__name__)

# --- Data Storage (In-memory) ---
MAX_HISTORY = 10
sensor_data_history = deque(maxlen=MAX_HISTORY)
latest_data = {}
latest_prediction = "N/A"  # Variable to store the latest prediction

# --- Load the Pre-trained Model ---
MODEL_PATH = 'Server\stress_model.pkl'
# Define feature columns using names EXACTLY as seen during training
# Based on the previous error message
feature_cols_expected_by_model = [
    'hrv',          # Example: This might come from rmssd or sdnn
    'light_lux',    # Example: This might come from a calibrated light percentage
    # Example: This might come from MPU6050 data (not currently sent)
    'movement',
    'noise_db',     # Example: This might come from the 'noise' amplitude
    'temperature_c',  # Example: This might come from 'temperature'
    # Add ALL other columns your model was trained on, in the correct order
    'humidity'      # Example: Assuming humidity was also used in training
]

try:
    model = joblib.load(MODEL_PATH)
    print(f"Machine learning model loaded successfully from {MODEL_PATH}")
except FileNotFoundError:
    print(
        f"Error: Model file not found at {MODEL_PATH}. Please run train_model.py.")
    model = None  # Set model to None if loading failed
except Exception as e:
    print(f"Error loading the model: {e}")
    model = None

# --- Optional: MQTT Configuration (for sending commands) ---
# MQTT_BROKER_HOST = "localhost"
# MQTT_BROKER_PORT = 1883
# MQTT_COMMAND_TOPIC = "emogotchi/window/command"
# STRESS_LEVEL_TO_TRIGGER = "Stressed"
# mqtt_client = None
# def setup_mqtt():
#     global mqtt_client
#     try:
#         mqtt_client = mqtt.Client(client_id="flask_server_ml")
#         mqtt_client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
#         mqtt_client.loop_start()
#         print("MQTT Client configured and connected.")
#         return True
#     except Exception as e:
#         print(f"Could not connect to MQTT Broker: {e}")
#         mqtt_client = None
#         return False
# mqtt_connected = setup_mqtt()
# --- End Optional MQTT ---


# --- Route to Serve the Webpage ---
@app.route('/')
def index():
    # Renders the index.html file from a 'templates' folder
    return render_template('index.html')

# --- Route to Provide Data AND Prediction to the Webpage ---


@app.route('/data')
def get_data():
    # Return the latest data, history, and prediction as JSON
    return jsonify({
        "latest": latest_data,
        "history": list(sensor_data_history),  # Convert deque to list for JSON
        "prediction": latest_prediction
    })

# --- Endpoint to Receive Data, Predict, and Optionally Command ---


@app.route('/api/upload', methods=['POST'])
def receive_data():
    global latest_data, latest_prediction  # Allow modification
    if not request.is_json:
        print("Error: Request was not JSON")
        return jsonify({"status": "error", "message": "Request must be JSON"}), 400

    # Get the JSON data sent by the ESP32
    data = request.get_json()
    print("Received data:")
    print(json.dumps(data, indent=4))

    # --- Store the data ---
    data['timestamp'] = time.strftime('%H:%M:%S')
    latest_data = data
    sensor_data_history.append(data)
    # --------------------

    # --- Machine Learning Prediction ---
    prediction_result = "N/A"  # Default prediction
    if model:  # Only predict if the model loaded successfully
        try:
            # --- Map incoming JSON keys to the feature names expected by the model ---
            input_data_mapped = {}

            # --- YOU NEED TO ADD/ADJUST MAPPINGS BELOW based on your ESP32 JSON and model needs ---
            # Example Mappings:
            input_data_mapped['temperature_c'] = data.get(
                'temperature', 0)  # Use 0 if 'temperature' key is missing
            # Use 0 if 'humidity' key is missing
            input_data_mapped['humidity'] = data.get('humidity', 0)
            # Use 0 if 'noise' key is missing
            input_data_mapped['noise_db'] = data.get('noise', 0)

            # HRV Mapping (Choose one or combine rmssd/sdnn as needed by your model)
            # Example: Using RMSSD. Change if model expects SDNN or something else. Use 0 if missing.
            input_data_mapped['hrv'] = data.get('rmssd', 0)

            # Light Mapping (Assuming ESP sends 'light_percent'. Use 0 if missing.)
            input_data_mapped['light_lux'] = data.get('light_percent', 0)

            # Movement Mapping (Currently missing from ESP data. Sending 0)
            # You'll need to send relevant MPU data (e.g., std dev of acceleration) from ESP later
            input_data_mapped['movement'] = data.get(
                'movement_metric', 0)  # Use 0 for now

            # --- Make sure ALL columns in feature_cols_expected_by_model are mapped ---

            # Fill any remaining expected columns that weren't mapped with a default (e.g., 0)
            for col in feature_cols_expected_by_model:
                if col not in input_data_mapped:
                    print(
                        f"Warning: Expected feature '{col}' not found in mapping. Filling with 0.")
                    input_data_mapped[col] = 0

            # 1. Create a pandas DataFrame using the CORRECT column names and order
            # Ensure the DataFrame only contains and is ordered by the expected columns
            input_df = pd.DataFrame([input_data_mapped])[
                feature_cols_expected_by_model]

            # 2. Make prediction
            prediction = model.predict(input_df)
            prediction_result = prediction[0]  # Get the prediction string

            print(f"Prediction: {prediction_result}")

            # --- Optional: Send MQTT command based on prediction ---
            # if prediction_result == STRESS_LEVEL_TO_TRIGGER and mqtt_connected and mqtt_client and mqtt_client.is_connected():
            #     command = "OPEN"
            #     mqtt_client.publish(MQTT_COMMAND_TOPIC, command)
            #     print(f"Published command '{command}' to topic '{MQTT_COMMAND_TOPIC}'")
            # --- End Optional MQTT ---

        except KeyError as e:
            print(
                f"Error during prediction mapping: Missing key {e} in received data or mapping.")
            prediction_result = "Mapping Error"
        except Exception as e:
            print(f"Error during prediction execution: {e}")
            prediction_result = "Prediction Error"
    else:
        print("Model not loaded, skipping prediction.")
        prediction_result = "N/A - Model Error"

    latest_prediction = prediction_result  # Update global state for UI
    # --------------------------------

    # Send success response back to the ESP32
    return jsonify({"status": "success", "message": "Data received"}), 200


# Run the Flask development server
if __name__ == '__main__':
    # use_reloader=False helps prevent issues with model/MQTT setup in debug mode
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
