#include "Arduino.h"

// Using built-in LED pin for heartbeat indication
#define ledPin 2

// Pulse meter connected to an Analog pin (GPIO 32 is a valid ADC pin)
#define sensorPin 32

// This is the minimum change in signal that will be considered a heartbeat.
// It acts as a noise filter. If still get false readings, try increasing
#define MIN_CHANGE 7.5

float thresholdMax = 0.0;
// Values for the filter and loop delay
float alpha = 0.75;
int period = 50;

void setup() {

  // Inbuilt LED
  pinMode(ledPin, OUTPUT);

  // Debugging window
  Serial.begin(9600);
  Serial.println("Pulse rate detection started.");

}

// ------------------------------------------------------------
// LOOP     LOOP     LOOP     LOOP     LOOP     LOOP     LOOP
// ------------------------------------------------------------
void loop() {

  // Arbitrary initial value for the sensor value
  static float oldValue = 500;

  // Time recording for BPM (beats per minute)
  static unsigned long bpmMills = millis();
  static int bpm = 0;

	// Keep track of the firstbeat recording time
	static unsigned long FirstBpmTime = millis();

  // Keep track of when we had the the last pulse - ignore
  // further pulses if too soon (probably false reading)
  static unsigned long timeBetweenBeats = millis();
  int minDelayBetweenBeats = 400;

  // Read the raw sensor value (0 - 4095 on ESP32)
  int rawValue = analogRead(sensorPin);

  // A low-pass filter to smooth the signal
  float value = alpha * oldValue + (1 - alpha) * rawValue;
  float change = value - oldValue;
  oldValue = value;

	// To indicate no finger is on the sensor for heartbeat measurment
	if (change < MIN_CHANGE && (millis() - timeBetweenBeats) >= 2000) {
		bpm = 0;
	}

  // A beat is now only detected if the change is greater than the dynamic threshold AND
  // it's also greater than our fixed noise filter threshold (MIN_CHANGE).
  if ((change >= thresholdMax) && (change > MIN_CHANGE)  && (change < 30) && (millis() > timeBetweenBeats + minDelayBetweenBeats)) {
		if (bpm == 0){
			FirstBpmTime = millis();
		}

    // Reset thresholdMax every time we find a new peak
    thresholdMax = change;

    // Flash LED
    digitalWrite(ledPin, HIGH);

    // Reset the heart beat time values
    timeBetweenBeats = millis();
    bpm++;
    Serial.print("Heart beat detected!\n");
  }
  else {
    // No pulse detected, ensure LED is off
    digitalWrite(ledPin, LOW);
  }

  // Slowly decay thresholdMax so it can adapt to signal changes
  thresholdMax = thresholdMax * 0.97;

  // Every 15 seconds, calculate and print the approximate pulse rate
  if (millis() >= bpmMills + 15000) {
    Serial.print("BPM (approx): ");
		if (bpm == 0) {
			Serial.println("--");
		} else {
			Serial.println(bpm * (60000 / (timeBetweenBeats - FirstBpmTime))); // Calculates bpm based on the bpm counts over the recorded timing of first and last beat
		}
    
    bpm = 0;
    bpmMills = millis();
  }

  // Delay to control the sampling rate
  delay(period);
}

