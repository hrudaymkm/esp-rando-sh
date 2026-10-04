
/*
  ESP32 Material Handler Controller
*/

#include <LittleFS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <MotorControl.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================================================
// Configuration
// =====================================================

// Turret motor
constexpr uint8_t TURRET_IN1 = 32;
constexpr uint8_t TURRET_IN2 = 33;

// Motor driver sleep/enable
constexpr uint8_t MOTOR_SLEEP_PIN = 25;

// Turret speed
constexpr int TURRET_MIN_SPEED = 100;
constexpr int TURRET_MAX_SPEED = 220;
constexpr int TURRET_SPEED_STEP = 40;
constexpr uint32_t TURRET_RAMP_INTERVAL = 20;

// WiFi
constexpr char WIFI_SSID[] = "SSID";
constexpr char WIFI_PASSWORD[] = "PASS";

// WebSocket
constexpr char WS_PATH[] = "/ws";

// OLED
constexpr uint8_t OLED_SDA = 26;
constexpr uint8_t OLED_SCL = 27;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;

// =====================================================
// Objects
// =====================================================

MotorControl turretMotor(TURRET_IN1, TURRET_IN2);

AsyncWebServer server(80);
AsyncWebSocket ws(WS_PATH);

Adafruit_SSD1306 display(
  OLED_WIDTH,
  OLED_HEIGHT,
  &Wire,
  -1
);

// =====================================================
// Turret state
// =====================================================

int turretCurrentSpeed = 0;
int turretTargetSpeed = 0;

bool turretActive = false;

uint32_t lastTurretRamp = 0;

// =====================================================
// OLED state
// =====================================================

bool lastOLEDConnection = false;

int lastOLEDTarget = 9999;
int lastOLEDCurr = 9999;
bool lastOLEDSleep = false;

// =====================================================
// Turret control
// =====================================================

void turretLeft() {
  turretActive = true;

  if (turretCurrentSpeed == 0)
    turretCurrentSpeed = TURRET_MIN_SPEED;

  turretTargetSpeed = TURRET_MAX_SPEED;

  Serial.println("Turret Left");
}

void turretRight() {
  turretActive = true;

  if (turretCurrentSpeed == 0)
    turretCurrentSpeed = -TURRET_MIN_SPEED;

  turretTargetSpeed = -TURRET_MAX_SPEED;

  Serial.println("Turret Right");
}

void turretStop() {
  turretActive = false;
  turretTargetSpeed = 0;

  Serial.println("Turret Stop");
}

void updateTurretMotor() {
  uint32_t now = millis();

  if (now - lastTurretRamp < TURRET_RAMP_INTERVAL)
    return;

  lastTurretRamp = now;

  if (turretCurrentSpeed < turretTargetSpeed) {
    turretCurrentSpeed += TURRET_SPEED_STEP;

    if (turretCurrentSpeed > turretTargetSpeed)
      turretCurrentSpeed = turretTargetSpeed;
  }
  else if (turretCurrentSpeed > turretTargetSpeed) {
    turretCurrentSpeed -= TURRET_SPEED_STEP;

    if (turretCurrentSpeed < turretTargetSpeed)
      turretCurrentSpeed = turretTargetSpeed;
  }

  turretMotor.setSpeed(turretCurrentSpeed);

  bool motorsActive =
    turretCurrentSpeed != 0 ||
    turretTargetSpeed != 0;

  digitalWrite(
    MOTOR_SLEEP_PIN,
    motorsActive ? HIGH : LOW
  );

  if (!turretActive && turretCurrentSpeed == 0)
    turretMotor.coast();
}

// =====================================================
// OLED
// =====================================================

void oledShowIP() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("IP:");

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.println(WiFi.localIP());

  display.display();
}

void oledShowTurret() {
  bool sleepState = digitalRead(MOTOR_SLEEP_PIN);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("TURRET");

  display.setTextSize(1);

  display.setCursor(0, 27);
  display.print("Target:  ");
  display.println(turretTargetSpeed);

  display.setCursor(0, 39);
  display.print("Current: ");
  display.println(turretCurrentSpeed);

  display.setCursor(0, 51);
  display.print("Sleep:   ");
  display.println(sleepState ? "ON" : "OFF");

  display.display();
}

void updateOLED() {
  bool connected = ws.count() > 0;

  // Connection state changed
  if (connected != lastOLEDConnection) {
    lastOLEDConnection = connected;

    if (connected) {
      lastOLEDTarget = 9999;
      lastOLEDCurr = 9999;
      lastOLEDSleep = !digitalRead(MOTOR_SLEEP_PIN);

      oledShowTurret();
    }
    else {
      oledShowIP();
    }

    return;
  }

  // No client connected
  if (!connected)
    return;

  int target = turretTargetSpeed;
  int current = turretCurrentSpeed;
  bool sleepState = digitalRead(MOTOR_SLEEP_PIN);

  if (
    target != lastOLEDTarget ||
    current != lastOLEDCurr ||
    sleepState != lastOLEDSleep
  ) {
    lastOLEDTarget = target;
    lastOLEDCurr = current;
    lastOLEDSleep = sleepState;

    oledShowTurret();
  }
}

// =====================================================
// WebSocket
// =====================================================

void onWebSocketEvent(
  AsyncWebSocket* server,
  AsyncWebSocketClient* client,
  AwsEventType type,
  void* arg,
  uint8_t* data,
  size_t len
) {

  switch (type) {

    case WS_EVT_CONNECT:
      Serial.printf(
        "Client #%u connected\n",
        client->id()
      );

      break;

    case WS_EVT_DISCONNECT:
      Serial.printf(
        "Client #%u disconnected\n",
        client->id()
      );

      turretStop();
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo* info =
        (AwsFrameInfo*)arg;

      if (
        !info->final ||
        info->index != 0 ||
        info->len != len ||
        info->opcode != WS_TEXT
      ) {
        break;
      }

      String cmd;
      cmd.reserve(len);

      for (size_t i = 0; i < len; i++)
        cmd += (char)data[i];

      // System stop
      if (
        cmd == "system:stop" ||
        cmd == "system:stopall"
      ) {
        turretStop();
        break;
      }

      // Split command
      int separator = cmd.indexOf(':');

      if (separator < 0)
        break;

      String group =
        cmd.substring(0, separator);

      String action =
        cmd.substring(separator + 1);

      // Turret commands
      if (group == "turret") {

        if (action == "left")
          turretLeft();

        else if (action == "right")
          turretRight();

        else if (action == "stop")
          turretStop();
      }

      break;
    }

    default:
      break;
  }
}

// =====================================================
// Web server
// =====================================================

void initWebServer() {
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET,
    [](AsyncWebServerRequest* request) {
      request->redirect("/main.html");
    }
  );

  server.serveStatic("/", LittleFS, "/");

  server.onNotFound(
    [](AsyncWebServerRequest* request) {
      request->send(
        404,
        "text/plain",
        "404 - File Not Found"
      );
    }
  );

  server.begin();

  Serial.println();
  Serial.println("===============================");
  Serial.println("||  Material Handler Ready   ||");
  Serial.println("===============================");
  Serial.print("||  IP Address : ");
  Serial.print(WiFi.localIP());
  Serial.println("  ||");
  Serial.println("===============================");
}

// =====================================================
// WiFi
// =====================================================

void connectWiFi() {
  Serial.print("Connecting to WiFi");

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());
}

// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  // Motor
  pinMode(MOTOR_SLEEP_PIN, OUTPUT);
  digitalWrite(MOTOR_SLEEP_PIN, LOW);

  turretMotor.begin();

  // OLED
  Wire.begin(
    OLED_SDA,
    OLED_SCL
  );

  if (!display.begin(
    SSD1306_SWITCHCAPVCC,
    OLED_ADDR
  )) {
    Serial.println("OLED initialization failed!");
  }

  // LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed.");

    while (true)
      delay(1000);
  }

  // WiFi
  connectWiFi();

  // Web server
  initWebServer();

  // Initial OLED screen
  oledShowIP();
}

// =====================================================
// Loop
// =====================================================

void loop() {
  ws.cleanupClients();

  updateTurretMotor();
  updateOLED();

  delay(1);
}
