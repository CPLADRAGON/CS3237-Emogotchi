import google.generativeai as genai

GOOGLE_API_KEY = "AIzaSyA5jjRmpcW1GHpkYW6tOcuuCeIK2NxsIKQ"  # Your key from server.py
genai.configure(api_key=GOOGLE_API_KEY)

print("Listing available models...")
try:
    for m in genai.list_models():
        if 'generateContent' in m.supported_generation_methods:
            print(f"- {m.name}")
except Exception as e:
    print(f"Error: {e}")
