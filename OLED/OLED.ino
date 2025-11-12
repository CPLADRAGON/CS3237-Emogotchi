#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Define your display specifications ---
#define SCREEN_WIDTH 64  // OLED display width, in pixels
#define SCREEN_HEIGHT 48 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if not used)

// --- I2C Address ---
// This is the most common address. If it doesn't work, try 0x3D.
#define SCREEN_ADDRESS 0x3C 

// Create the display object.
// For ESP32, Wire (I2C) defaults to GPIO 21 (SDA) and GPIO 22 (SCL)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200); // Start serial for debugging

  // Initialize the display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    // If it fails, stop here.
    for(;;); 
  }

  // --- If successful, draw to the display ---
  
  // 1. Clear the internal buffer
  display.clearDisplay();

  // 2. Set text properties
  display.setTextSize(1);      // Set text size (1 is smallest)
  display.setTextColor(SSD1306_WHITE); // Set text color to white
  display.setCursor(10, 40);   // Set (x, y) position to start drawing

  // 3. Add text to the buffer
  display.println(F("Hello!"));

  // 4. Send the buffer to the display to make it visible
  display.display(); 
}

void loop() {
  // Since we're just displaying static text, 
  // there's nothing to do in the loop.
}