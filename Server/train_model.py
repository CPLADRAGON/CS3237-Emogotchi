import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score
import joblib  # Library to save/load scikit-learn models

# --- Configuration ---
DATASET_PATH = 'simulated_emogotchi_dataset_v3.csv'
MODEL_SAVE_PATH = 'stress_model.pkl'

# Define the features (sensor readings) and the target (stress level)
# --- IMPORTANT: Adjust these column names to match your CSV file exactly ---
# Example features, ADD ALL YOUR SENSOR COLUMNS
feature_cols = ['bpm', 'temperature', 'humidity', 'noise', ]
target_col = 'Emotion'  # The column with 'Relaxed', 'Neutral', 'Stressed' labels

# --- Load Data ---
try:
    df = pd.read_csv(DATASET_PATH)
    print(f"Dataset loaded successfully from {DATASET_PATH}")
    print(f"Dataset shape: {df.shape}")
    print("\nFirst 5 rows:\n", df.head())
    print("\nStress Level distribution:\n", df[target_col].value_counts())

    # --- Data Cleaning (Example: Handle missing values if any) ---
    df = df.dropna(subset=feature_cols + [target_col])
    print(f"Dataset shape after dropping NaNs: {df.shape}")

    # Check if dataset is empty after cleaning
    if df.empty:
        raise ValueError(
            "Dataset is empty after cleaning. Check your CSV file and feature/target columns.")

except FileNotFoundError:
    print(f"Error: Dataset file not found at {DATASET_PATH}")
    exit()
except KeyError as e:
    print(
        f"Error: Column '{e}' not found in the dataset. Please check feature_cols and target_col.")
    exit()
except Exception as e:
    print(f"An unexpected error occurred during data loading: {e}")
    exit()

# --- Prepare Data ---
X = df[feature_cols]  # Features
y = df[target_col]   # Target variable (labels)

# Split dataset into training set and test set
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.3, random_state=42)  # 70% training and 30% test

# --- Train Random Forest Model ---
# Create a Gaussian Classifier
# You can tune parameters like n_estimators, max_depth, etc.
clf = RandomForestClassifier(n_estimators=100, random_state=42)

print("\nTraining the Random Forest model...")
# Train the model using the training sets
clf.fit(X_train, y_train)
print("Model training complete.")

# --- Evaluate Model ---
# Predict the response for test dataset
y_pred = clf.predict(X_test)

# Model Accuracy, how often is the classifier correct?
accuracy = accuracy_score(y_test, y_pred)
print(f"\nModel Accuracy on Test Set: {accuracy * 100:.2f}%")

# --- Save the Trained Model ---
try:
    joblib.dump(clf, MODEL_SAVE_PATH)
    print(f"Trained model saved successfully to {MODEL_SAVE_PATH}")
except Exception as e:
    print(f"Error saving the model: {e}")

# --- Optional: Show Feature Importance ---
print("\nFeature Importances:")
importances = pd.Series(clf.feature_importances_,
                        index=feature_cols).sort_values(ascending=False)
print(importances)
