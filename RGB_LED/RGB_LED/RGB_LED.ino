/*
  RGB LED Status Display
  
  Manages 3 "interesting" light patterns for 3 states:
  - Stressed: A fast, blinking red "alarm."
  - Normal: A solid, calm blue.
  - Happy: A slow, "breathing" green light.

  This code is non-blocking (uses millis() instead of delay()) 
  so it can be integrated into your main FreeRTOS project without
  stopping other tasks.
*/

// --- Pins ---
// Your pins from the example
int led_red = 12;   // Pin for red (WAS 13)
int led_green = 13; // Pin for green (WAS 12)
int led_blue = 14;  // Pin for blue

// --- State Management ---
// In your main project, your MQTT callback will change this variable.
String currentState = "happy"; // Can be "normal", "stressed", or "happy"

// --- Timing & Animation Variables ---
// These are 'static' so they remember their value between loops

// For "Happy" state (Breathing)
static float happyPulseSpeed = 0.005; // Controls the "breathing" speed

// For "Stressed" state (Blinking)
static unsigned long stressedLastUpdate = 0;
static int stressedBlinkSpeed = 250; // 250ms on, 250ms off
static bool stressedState = false; // false = off, true = on

void setup() {
  // Initialization of output pins for the LEDs
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(led_blue, OUTPUT);

  // Initialize Serial to test (you can remove this later)
  Serial.begin(115200);
  
  // No longer needed for 'happy'
  // randomSeed(analogRead(34)); 
}

void loop() {
  /*
    In your main ESP32 project, this 'loop()' function will
    be called repeatedly (or you can put this logic in a
    dedicated FreeRTOS task).
    
    Your MQTT callback function will just set the 'currentState' string.
    The loop will then automatically handle the light pattern.
  */

  // --- START - Mock code to test patterns ---
  // You can uncomment one of these lines to test a pattern
  // currentState = "stressed";
  // currentState = "happy";
  // currentState = "normal";
  // --- END - Mock code ---


  // Check the current state and call the matching function
  if (currentState == "stressed") {
    showStressed();
  } else if (currentState == "happy") {
    showHappy();
  } else {
    // Default to "normal"
    showNormal();
  }
}

// --- Helper function to set all 3 colors ---
void showRGB(int r, int g, int b) {
  // Constrain values to be 0-255
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);

  analogWrite(led_red, r);
  analogWrite(led_green, g);
  analogWrite(led_blue, b);
}

// --- STATE 1: "Stressed" ---
// A fast, blinking red light.
void showStressed() {
  // Check if it's time to toggle the blink
  if (millis() - stressedLastUpdate > stressedBlinkSpeed) {
    stressedLastUpdate = millis(); // Remember the time
    stressedState = !stressedState; // Flip the state (on to off, or off to on)

    if (stressedState) {
      showRGB(255, 0, 0); // LED On (Bright Red)
    } else {
      showRGB(0, 0, 0); // LED Off
    }
  }
}

// --- STATE 2: "Normal" ---
// A solid, calm blue.
void showNormal() {
  // Just show a solid blue color
  showRGB(0, 0, 255);
}

// --- STATE 3: "Happy" ---
// A slow, "breathing" green light.
void showHappy() {
  // Create a pulsing value between 0.0 and 1.0 using a sine wave
  float pulse = (sin(millis() * happyPulseSpeed) + 1.0) / 2.0;
  
  // Map the pulse from a dim green (50) to a bright green (255)
  int greenValue = 50 + (pulse * 205);
  
  showRGB(0, greenValue, 0);
}