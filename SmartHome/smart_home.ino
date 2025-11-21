#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

const char *ssid = "CPLADRAGON";
const char *password = "10293847";
const char *mqtt_server = "178.128.94.113";
const char *mqtt_command_topic = "esp32/prediction";

// OLED (I2C)
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 48

// Status Pulse Output
#define STRESS_PIN_0 18
#define STRESS_PIN_1 17

// Inputs (Buttons) - using INPUT_PULLUP
#define DOOR_PIN 25
#define WINDOWS_PIN 26
#define RGB_PIN 27
#define RELAX_PIN 32

// Outputs
#define LED_RED 12
#define LED_GREEN 13
#define LED_BLUE 14
#define DOOR_OUT_PIN 15
#define WINDOWS_OUT_PIN 16

// --- OBJECTS ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
WiFiClient espClient;
PubSubClient client(espClient);
Servo door;
Servo windows;

// MQTT/Score
volatile float g_happinessScore = 50.0;
volatile bool g_newScoreAvailable = false;
volatile bool g_sadStateTriggered = false; //
volatile bool g_relaxModeActive = false;

static const unsigned char PROGMEM icon_normal_32x32[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x0f, 0xff, 0xf0, 0x00,
    0x1f, 0xff, 0xf8, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xfe, 0x00,
    0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0x80, 0xff, 0x03, 0xff, 0x80,
    0xff, 0x00, 0xff, 0x80, 0xff, 0x00, 0xff, 0x80, 0x7f, 0x00, 0x7e, 0x00, 0x7f, 0x00, 0x7e, 0x00,
    0x7f, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x1f, 0xff, 0xf8, 0x00,
    0x0e, 0x00, 0x70, 0x00, 0x0c, 0x00, 0x60, 0x00, 0x0c, 0xff, 0x60, 0x00, 0x0c, 0xff, 0x60, 0x00,
    0x0c, 0x00, 0x60, 0x00, 0x0e, 0x00, 0x70, 0x00, 0x1f, 0xff, 0xf8, 0x00, 0x3f, 0xff, 0xfc, 0x00,
    0x7f, 0xff, 0xfe, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const unsigned char PROGMEM icon_happy_32x32[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x0f, 0xff, 0xf0, 0x00,
    0x1f, 0xff, 0xf8, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xfe, 0x00,
    0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0x80, 0xff, 0x03, 0xff, 0x80,
    0xff, 0x00, 0xff, 0x80, 0xff, 0x00, 0xff, 0x80, 0x7f, 0x00, 0x7e, 0x00, 0x7f, 0x00, 0x7e, 0x00,
    0x7f, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x1f, 0xff, 0xf8, 0x00,
    0x0e, 0x00, 0x70, 0x00, 0x0c, 0x00, 0x60, 0x00, 0x0c, 0xff, 0x60, 0x00, 0x06, 0xff, 0xc0, 0x00,
    0x06, 0x00, 0xc0, 0x00, 0x0e, 0x00, 0x70, 0x00, 0x1f, 0x80, 0xf8, 0x00, 0x3f, 0xc1, 0xfc, 0x00,
    0x7f, 0xff, 0xfe, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const unsigned char PROGMEM icon_sad_32x32[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x0f, 0xff, 0xf0, 0x00,
    0x1f, 0xff, 0xf8, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xfe, 0x00,
    0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0x80, 0xff, 0x03, 0xff, 0x80,
    0xff, 0x00, 0xff, 0x80, 0xff, 0x00, 0xff, 0x80, 0x7f, 0x00, 0x7e, 0x00, 0x7f, 0x00, 0x7e, 0x00,
    0x7f, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x1f, 0xff, 0xf8, 0x00,
    0x0e, 0x00, 0x70, 0x00, 0x0c, 0x00, 0x60, 0x00, 0x0c, 0xff, 0x60, 0x00, 0x0c, 0xff, 0x60, 0x00,
    0x0e, 0x00, 0x70, 0x00, 0x1f, 0x80, 0xf8, 0x00, 0x3f, 0xc1, 0xfc, 0x00, 0x0e, 0x00, 0x70, 0x00,
    0x06, 0x00, 0xc0, 0x00, 0x06, 0xff, 0xc0, 0x00, 0x0c, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};

void setup_wifi();
void reconnectMQTT();
void mqttCallback(char *topic, byte *payload, unsigned int length);
void updateOLED(float score);
void showRGB(int r, int g, int b);
void Relax_mode();

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting Smart Home MQTT Subscriber...");
  setup_wifi();
  setup_oled();

  pinMode(STRESS_PIN_0, OUTPUT);
  pinMode(STRESS_PIN_1, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(STRESS_PIN_0, LOW);
  digitalWrite(STRESS_PIN_1, LOW);

  // --- 2. Configure Input Pins ---
  pinMode(DOOR_PIN, INPUT_PULLDOWN);
  pinMode(WINDOWS_PIN, INPUT_PULLDOWN);
  pinMode(RGB_PIN, INPUT_PULLDOWN);
  pinMode(RELAX_PIN, INPUT_PULLDOWN);

  // Servos
  door.attach(DOOR_OUT_PIN);
  windows.attach(WINDOWS_OUT_PIN);

  // MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi disconnected, Reconnecting...");
    setup_wifi();
  }

  if (!client.connected())
  {
    reconnectMQTT();
  }
  client.loop();

  if (g_newScoreAvailable)
  {
    g_newScoreAvailable = false;

    Serial.printf("Main Loop: 收到新分数: %.2f\n", g_happinessScore);

    updateOLED(g_happinessScore);

    // update STRESS_PIN accroding to g_happinessScore
    if (g_happinessScore <= 33.0)
    {
      //
      // check if sad unrecovered (g_sadStateTriggered = false)
      if (!g_sadStateTriggered)
      {
        Serial.println("State Change: SAD. Sending trigger ONCE.");
        digitalWrite(STRESS_PIN_0, HIGH);
        g_sadStateTriggered = true;
      }
      digitalWrite(STRESS_PIN_1, LOW); // ASR Pin 1
    }
    else if (g_happinessScore <= 67.0)
    {
      Serial.println("State Change: NORMAL. Resetting trigger.");
      digitalWrite(STRESS_PIN_0, LOW);
      digitalWrite(STRESS_PIN_1, HIGH);
      g_sadStateTriggered = false;
    }
    else
    {
      Serial.println("State Change: HAPPY. Resetting trigger.");
      digitalWrite(STRESS_PIN_0, LOW);
      digitalWrite(STRESS_PIN_1, LOW);
      g_sadStateTriggered = false;
    }
  }

  if (digitalRead(RELAX_PIN) == LOW && g_happinessScore < 34)
  {
    g_relaxModeActive = true;
  }
  if (g_happinessScore >= 34)
  {
    g_relaxModeActive = false;
  }
  // RGB_PIN check
  if (g_relaxModeActive)
  {
    Relax_mode();
  }
  else if (digitalRead(RGB_PIN) == HIGH)
  {
    showRGB(255, 255, 255);
  }
  else
  {
    showRGB(0, 0, 0);
  }

  if (digitalRead(DOOR_PIN) == HIGH)
  {
    door.write(180);
  }
  else
  {
    door.write(0);
  }

  if (digitalRead(WINDOWS_PIN) == HIGH)
  {
    windows.write(180);
  }
  else
  {
    windows.write(0);
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  // payload to String
  String message = "";
  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }

  Serial.printf("\nMQTT Callback: Topic [%s]: %s\n", topic, message.c_str());

  // subscribe
  if (String(topic) == mqtt_command_topic)
  {

    // resolve Emotion:Score
    int separatorIndex = message.indexOf(':');

    if (separatorIndex != -1)
    {
      String emotion = message.substring(0, separatorIndex);
      String scoreStr = message.substring(separatorIndex + 1);

      // update global flag
      g_happinessScore = scoreStr.toFloat();
      g_newScoreAvailable = true;

      Serial.printf("Callback: Parsed Emotion: %s, Score: %.2f\n", emotion.c_str(), g_happinessScore);
    }
    else
    {
      Serial.println("Callback Error: Message format unknown (expected 'Emotion:Score')");
    }
  }
}

void setup_wifi()
{
  delay(10);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void setup_oled()
{
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("System Ready!");
    display.display();
  }
  else
  {
    Serial.println("SSD1306 allocation failed");
  }
}

void reconnectMQTT()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("esp32-SmartHome-Device"))
    {
      Serial.println("connected");

      if (client.subscribe(mqtt_command_topic))
      {
        Serial.printf("Subscribed to command topic: %s\n", mqtt_command_topic);
      }
      else
      {
        Serial.println("Failed to subscribe to command topic!");
      }
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void updateOLED(float score)
{
  display.clearDisplay();
  display.setCursor(0, 0);

  const unsigned char *iconToDraw;

  ////////////////////////////////////////////smilling face
  /**/
  if (score <= 33.0)
  {
    iconToDraw = icon_sad_32x32;
  }
  else if (score <= 67.0)
  {
    iconToDraw = icon_normal_32x32;
  }
  else
  {
    iconToDraw = icon_happy_32x32;
  }

  display.drawBitmap(16, 8, iconToDraw, 32, 32, SSD1306_WHITE);

  char scoreBuffer[10];
  snprintf(scoreBuffer, sizeof(scoreBuffer), "%.1f", score);
  display.setTextSize(1);
  display.setCursor(18, 40);
  display.print(scoreBuffer);

  display.display();
}

void showRGB(int r, int g, int b)
{
  // Constrain values to be 0-255
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);

  // Use the #defined pins
  analogWrite(LED_RED, r);
  analogWrite(LED_GREEN, g);
  analogWrite(LED_BLUE, b);
}

void Relax_mode()
{
  float pulse = (sin(millis() * 0.002) + 1.0) / 2.0;
  int Value = 50 + (pulse * 205);
  showRGB(0, Value, Value); // Breathing
}