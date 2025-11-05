#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <Ticker.h>

// --- WiFi Configuration ---
const char *ssid = "";     // Replace with your WiFi SSID
const char *password = ""; // Replace with your WiFi password

// --- Web Server ---
ESP8266WebServer server(80);

// --- Configuration ---
#define ENA_PIN D5 // Connect L298N ENA to Wemos D5 (or another PWM capable pin)
#define IN1_PIN D6 // Connect L298N IN1 to Wemos D6
#define IN2_PIN D7 // Connect L298N IN2 to Wemos D7
// Track 2
#define ENA_PIN2 D3 // Connect L298N ENA to Wemos D1 (or another PWM capable pin)
#define IN1_PIN2 D1 // Connect L298N IN1 to Wemos D2
#define IN2_PIN2 D2 // Connect L298N IN2 to Wemos D3

// --- Momentum Parameters ---
const int MOMENTUM_STEP = 1;       // How much to change speed in each step (acceleration/deceleration rate)
const long MOMENTUM_INTERVAL = 20; // Milliseconds between speed updates (higher value = slower change)
const int PWM_MAX = 1023;          // Max PWM value for ESP8266 (0-1023)

// --- Waveform Generator Parameters ---
const float WAVEFORM_FREQ = 125.0;   // 125Hz PRF for trapezoidal mode
const int TRIANGLE_THRESHOLD = 300;  // Below this speed, use triangle waves
const int WAVEFORM_RESOLUTION = 100; // Steps per waveform cycle
const float SLEW_RATE_LIMIT = 50.0;  // Triangle wave slew rate (PWM units per ms)
Ticker waveformTicker;

// --- Track 1 Variables ---
int target_speed = 0;                // Desired speed (0 to PWM_MAX)
int current_speed = 0;               // Current speed (0 to PWM_MAX)
unsigned long last_update_time = 0;  // Timer for momentum
bool direction_forward = true;       // Current motor direction
bool use_waveform_generator = false; // Toggle between classic PWM and waveform generator

// --- Track 1 Brake System Variables ---
int brake_level = 0;                // 0=off, 1=light, 2=medium, 3=heavy, 4=emergency
bool brake_active = false;          // Is brake currently applied
unsigned long brake_start_time = 0; // When brake was applied

// --- Track 2 Variables ---
int target_speed2 = 0;                // Desired speed (0 to PWM_MAX)
int current_speed2 = 0;               // Current speed (0 to PWM_MAX)
unsigned long last_update_time2 = 0;  // Timer for momentum
bool direction_forward2 = true;       // Current motor direction
bool use_waveform_generator2 = false; // Toggle between classic PWM and waveform generator

// --- Track 2 Brake System Variables ---
int brake_level2 = 0;                // 0=off, 1=light, 2=medium, 3=heavy, 4=emergency
bool brake_active2 = false;          // Is brake currently applied
unsigned long brake_start_time2 = 0; // When brake was applied

// --- Shared Brake Constants ---
const int BRAKE_STEP_LIGHT = 2;       // Light brake deceleration rate
const int BRAKE_STEP_MEDIUM = 4;      // Medium brake deceleration rate
const int BRAKE_STEP_HEAVY = 8;       // Heavy brake deceleration rate
const int BRAKE_STEP_EMERGENCY = 20;  // Emergency brake deceleration rate// --- Waveform Generator Variables ---
volatile int waveform_step = 0;       // Current step in waveform cycle
volatile float triangle_value = 0.0;  // Current triangle wave value
volatile bool triangle_rising = true; // Triangle wave direction
volatile int current_pwm_output = 0;  // Actual PWM value being output
volatile unsigned long last_waveform_update = 0;

// --- Waveform Generation Functions ---
void IRAM_ATTR updateWaveform()
{
  // Only run waveform generator if enabled
  if (!use_waveform_generator)
  {
    return;
  }

  if (current_speed == 0)
  {
    current_pwm_output = 0;
    analogWrite(ENA_PIN, 0);
    return;
  }

  // Calculate time since last update
  unsigned long current_time = micros();
  float dt = (current_time - last_waveform_update) / 1000.0; // Convert to milliseconds
  last_waveform_update = current_time;

  if (current_speed < TRIANGLE_THRESHOLD)
  {
    // Generate slew-limited triangle wave for low speeds
    float target_change = SLEW_RATE_LIMIT * dt;

    if (triangle_rising)
    {
      triangle_value += target_change;
      if (triangle_value >= current_speed)
      {
        triangle_value = current_speed;
        triangle_rising = false;
      }
    }
    else
    {
      triangle_value -= target_change;
      if (triangle_value <= 0)
      {
        triangle_value = 0;
        triangle_rising = true;
      }
    }
    current_pwm_output = (int)triangle_value;
  }
  else
  {
    // Generate 125Hz trapezoidal wave for higher speeds
    float cycle_time = 1000.0 / WAVEFORM_FREQ; // Period in milliseconds
    float step_time = cycle_time / WAVEFORM_RESOLUTION;

    // Advance waveform step based on time
    static float accumulated_time = 0;
    accumulated_time += dt;

    while (accumulated_time >= step_time)
    {
      waveform_step = (waveform_step + 1) % WAVEFORM_RESOLUTION;
      accumulated_time -= step_time;
    }

    // Generate trapezoidal waveform
    // Rise time: 20% of cycle, Flat time: 60% of cycle, Fall time: 20% of cycle
    int rise_steps = WAVEFORM_RESOLUTION * 0.2;
    int flat_steps = WAVEFORM_RESOLUTION * 0.6;
    int fall_steps = WAVEFORM_RESOLUTION * 0.2;

    if (waveform_step < rise_steps)
    {
      // Rising edge
      current_pwm_output = (current_speed * waveform_step) / rise_steps;
    }
    else if (waveform_step < (rise_steps + flat_steps))
    {
      // Flat top
      current_pwm_output = current_speed;
    }
    else
    {
      // Falling edge
      int fall_step = waveform_step - rise_steps - flat_steps;
      current_pwm_output = current_speed - (current_speed * fall_step) / fall_steps;
    }
  }

  // Apply the generated waveform
  analogWrite(ENA_PIN, current_pwm_output);
}

// --- Helper Function to Set Direction ---
void setDirection(bool forward)
{
  if (forward)
  {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  }
  else
  {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
}

void setDirection2(bool forward)
{
  if (forward)
  {
    digitalWrite(IN1_PIN2, HIGH);
    digitalWrite(IN2_PIN2, LOW);
  }
  else
  {
    digitalWrite(IN1_PIN2, LOW);
    digitalWrite(IN2_PIN2, HIGH);
  }
}

// --- Web Server Functions ---
void handleRoot()
{
  Serial.println("Root request received");

  // Serve from SPIFFS only
  if (SPIFFS.exists("/index.html"))
  {
    Serial.println("Serving from SPIFFS");
    File file = SPIFFS.open("/index.html", "r");
    if (file)
    {
      server.streamFile(file, "text/html");
      file.close();
    }
    else
    {
      Serial.println("Error: Could not open index.html");
      server.send(500, "text/plain", "Error: Could not open index.html");
    }
  }
  else
  {
    Serial.println("Error: index.html not found in SPIFFS");
    server.send(404, "text/plain", "Error: index.html not found in SPIFFS. Please upload filesystem.");
  }
}

void handleControl()
{
  if (server.method() == HTTP_POST)
  {
    String body = server.arg("plain");

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error)
    {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }

    String command = doc["command"];

    if (command == "speed")
    {
      int speed = doc["value"];
      target_speed = constrain(speed, 0, PWM_MAX);
      Serial.print("Target speed set to: ");
      Serial.println(target_speed);
    }
    else if (command == "direction")
    {
      String dir = doc["value"];
      direction_forward = (dir == "forward");
      setDirection(direction_forward);
      Serial.print("Direction set to: ");
      Serial.println(direction_forward ? "Forward" : "Reverse");
    }
    else if (command == "emergency_stop")
    {
      target_speed = 0;
      Serial.println("Emergency stop activated!");
    }
    else if (command == "waveform_mode")
    {
      use_waveform_generator = doc["value"];
      Serial.print("Waveform generator mode: ");
      Serial.println(use_waveform_generator ? "ENABLED (Triangle/Trapezoidal)" : "DISABLED (Classic PWM)");

      // Reset waveform state when switching modes
      if (use_waveform_generator)
      {
        triangle_value = 0.0;
        triangle_rising = true;
        waveform_step = 0;
        last_waveform_update = micros();
      }
    }
    else if (command == "brake")
    {
      int level = doc["value"];
      brake_level = constrain(level, 0, 4);
      brake_active = (brake_level > 0);
      brake_start_time = millis();

      Serial.print("Track 1 Brake applied - Level: ");
      Serial.println(brake_level == 0 ? "OFF" : brake_level == 1 ? "LIGHT"
                                            : brake_level == 2   ? "MEDIUM"
                                            : brake_level == 3   ? "HEAVY"
                                                                 : "EMERGENCY");
    }
    // Track 2 Commands
    else if (command == "speed2")
    {
      int speed = doc["value"];
      target_speed2 = constrain(speed, 0, PWM_MAX);
      Serial.print("Track 2 Target speed set to: ");
      Serial.println(target_speed2);
    }
    else if (command == "direction2")
    {
      String dir = doc["value"];
      direction_forward2 = (dir == "forward");
      setDirection2(direction_forward2);
      Serial.print("Track 2 Direction set to: ");
      Serial.println(direction_forward2 ? "Forward" : "Reverse");
    }
    else if (command == "emergency_stop2")
    {
      target_speed2 = 0;
      Serial.println("Track 2 Emergency stop activated!");
    }
    else if (command == "waveform_mode2")
    {
      use_waveform_generator2 = doc["value"];
      Serial.print("Track 2 Waveform generator mode: ");
      Serial.println(use_waveform_generator2 ? "ENABLED (Triangle/Trapezoidal)" : "DISABLED (Classic PWM)");
    }
    else if (command == "brake2")
    {
      int level = doc["value"];
      brake_level2 = constrain(level, 0, 4);
      brake_active2 = (brake_level2 > 0);
      brake_start_time2 = millis();

      Serial.print("Track 2 Brake applied - Level: ");
      Serial.println(brake_level2 == 0 ? "OFF" : brake_level2 == 1 ? "LIGHT"
                                             : brake_level2 == 2   ? "MEDIUM"
                                             : brake_level2 == 3   ? "HEAVY"
                                                                   : "EMERGENCY");
    }

    server.send(200, "application/json", "{\"status\":\"ok\"}");
  }
  else
  {
    server.send(405, "text/plain", "Method not allowed");
  }
}

void handleStatus()
{
  JsonDocument doc;

  // Track 1 Status
  doc["current_speed"] = current_speed;
  doc["target_speed"] = target_speed;
  doc["direction_forward"] = direction_forward;
  doc["adc_step"] = map(target_speed, 0, PWM_MAX, 0, 16);
  doc["waveform_mode"] = use_waveform_generator;
  if (use_waveform_generator)
  {
    doc["current_pwm_output"] = current_pwm_output;
    doc["waveform_type"] = (current_speed < TRIANGLE_THRESHOLD) ? "triangle" : "trapezoidal";
  }
  doc["brake_level"] = brake_level;
  doc["brake_active"] = brake_active;

  // Track 2 Status
  doc["current_speed2"] = current_speed2;
  doc["target_speed2"] = target_speed2;
  doc["direction_forward2"] = direction_forward2;
  doc["adc_step2"] = map(target_speed2, 0, PWM_MAX, 0, 16);
  doc["waveform_mode2"] = use_waveform_generator2;
  doc["brake_level2"] = brake_level2;
  doc["brake_active2"] = brake_active2;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void setup()
{
  // Track 1 pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Track 2 pins
  pinMode(ENA_PIN2, OUTPUT);
  pinMode(IN1_PIN2, OUTPUT);
  pinMode(IN2_PIN2, OUTPUT);

  // Set the initial directions
  setDirection(direction_forward);
  setDirection2(direction_forward2);

  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("\nLoco8266 Train Controller Starting...");

  // Initialize SPIFFS
  if (!SPIFFS.begin())
  {
    Serial.println("SPIFFS initialization failed! Please upload filesystem.");
  }
  else
  {
    Serial.println("SPIFFS initialized successfully");

    // Check if index.html exists
    if (SPIFFS.exists("/index.html"))
    {
      Serial.println("index.html found in SPIFFS");
      File file = SPIFFS.open("/index.html", "r");
      Serial.print("File size: ");
      Serial.println(file.size());
      file.close();
    }
    else
    {
      Serial.println("index.html NOT found in SPIFFS - please upload filesystem");
    }
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30)
  {
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println();
    Serial.print("WiFi connected! IP address: ");
    Serial.println(WiFi.localIP());

    // Start mDNS
    if (MDNS.begin("loco8266"))
    {
      Serial.println("mDNS responder started - accessible at http://loco8266.local");
    }

    // Setup web server routes
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/control", HTTP_POST, handleControl);
    server.on("/api/status", HTTP_GET, handleStatus);

    // Start web server
    server.begin();
    Serial.println("Web server started");
    Serial.println("Open http://loco8266.local or http://" + WiFi.localIP().toString() + " in your browser");
  }
  else
  {
    Serial.println("\nFailed to connect to WiFi!");
  }

  // Initialize waveform generator
  last_waveform_update = micros();
  waveformTicker.attach_ms(1, updateWaveform); // Update waveform every 1ms (1kHz update rate)
  Serial.println("Waveform generator initialized - Triangle mode <300, Trapezoidal @125Hz >=300");
}

void loop()
{
  // Handle web server requests
  server.handleClient();
  MDNS.update();

  // --- Implement Momentum (Non-blocking) ---
  unsigned long current_time = millis();

  if (current_time - last_update_time >= MOMENTUM_INTERVAL)
  {
    // Handle braking first - brakes override throttle
    if (brake_active)
    {
      int brake_step = MOMENTUM_STEP; // Default deceleration

      switch (brake_level)
      {
      case 1:
        brake_step = BRAKE_STEP_LIGHT;
        break;
      case 2:
        brake_step = BRAKE_STEP_MEDIUM;
        break;
      case 3:
        brake_step = BRAKE_STEP_HEAVY;
        break;
      case 4:
        brake_step = BRAKE_STEP_EMERGENCY;
        break;
      }

      // Apply braking deceleration
      if (current_speed > 0)
      {
        current_speed -= brake_step;
        if (current_speed < 0)
          current_speed = 0;
      }
    }
    else
    {
      // Normal momentum control when not braking
      if (current_speed < target_speed)
      {
        current_speed += MOMENTUM_STEP;
        if (current_speed > target_speed)
          current_speed = target_speed; // Cap it
      }
      else if (current_speed > target_speed)
      {
        current_speed -= MOMENTUM_STEP;
        if (current_speed < target_speed)
          current_speed = target_speed; // Cap it
      }
    }

    // Apply PWM based on mode for Track 1
    if (use_waveform_generator)
    {
      // PWM Speed is handled by the waveform generator
      // The updateWaveform() function applies the appropriate waveform based on current_speed
    }
    else
    {
      // Classic PWM mode
      analogWrite(ENA_PIN, current_speed);
    }

    last_update_time = current_time; // Reset the timer
  }

  // --- Track 2 Momentum Control ---
  if (current_time - last_update_time2 >= MOMENTUM_INTERVAL)
  {
    // Handle braking first - brakes override throttle
    if (brake_active2)
    {
      int brake_step = MOMENTUM_STEP; // Default deceleration

      switch (brake_level2)
      {
      case 1:
        brake_step = BRAKE_STEP_LIGHT;
        break;
      case 2:
        brake_step = BRAKE_STEP_MEDIUM;
        break;
      case 3:
        brake_step = BRAKE_STEP_HEAVY;
        break;
      case 4:
        brake_step = BRAKE_STEP_EMERGENCY;
        break;
      }

      // Apply braking deceleration
      if (current_speed2 > 0)
      {
        current_speed2 -= brake_step;
        if (current_speed2 < 0)
          current_speed2 = 0;
      }
    }
    else
    {
      // Normal momentum control when not braking
      if (current_speed2 < target_speed2)
      {
        current_speed2 += MOMENTUM_STEP;
        if (current_speed2 > target_speed2)
          current_speed2 = target_speed2; // Cap it
      }
      else if (current_speed2 > target_speed2)
      {
        current_speed2 -= MOMENTUM_STEP;
        if (current_speed2 < target_speed2)
          current_speed2 = target_speed2; // Cap it
      }
    }

    // Apply PWM for Track 2 (currently only classic PWM mode)
    analogWrite(ENA_PIN2, current_speed2);

    last_update_time2 = current_time; // Reset the timer

    // Optional debugging (uncomment if needed)
    /*
    Serial.print("Track 1 - Current Speed: ");
    Serial.print(current_speed);
    Serial.print(" | Target Speed: ");
    Serial.print(target_speed);
    Serial.print(" | Direction: ");
    Serial.println(direction_forward ? "Forward" : "Reverse");
    */

    last_update_time = current_time; // Reset the timer
  }
}