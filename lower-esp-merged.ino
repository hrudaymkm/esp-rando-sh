/*
  ESP32 Material Handler Controller
  Merged single-file version
*/

#include <gpio_viewer.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <MotorControl.h>
#include <ServoControl.h>

// ---------- Configuration ----------
constexpr uint8_t DRIVE_IN1 = 27;
constexpr uint8_t DRIVE_IN2 = 26;
constexpr uint8_t TURRET_IN1 = 32;
constexpr uint8_t TURRET_IN2 = 33;
constexpr uint8_t STEERING_SERVO_PIN = 13;
constexpr uint8_t MOTOR_SLEEP_PIN = 25;

constexpr uint8_t LEFT_ANGLE = 20;
constexpr uint8_t CENTER_ANGLE = 90;
constexpr uint8_t RIGHT_ANGLE = 160;

constexpr int DRIVE_MIN_SPEED = 70;
constexpr int DRIVE_MAX_SPEED = 200;
constexpr int DRIVE_STEP = 5;

constexpr int TURRET_MIN_SPEED = 90;
constexpr int TURRET_MAX_SPEED = 255;
constexpr int TURRET_STEP = 8;

constexpr uint16_t WS_TIMEOUT_MS = 3000;

constexpr char WIFI_SSID[] = "<wifi ssid>";
constexpr char WIFI_PASSWORD[] = "<wifi pass>";

// ---------- Hardware ----------
MotorControl driveMotor(DRIVE_IN1, DRIVE_IN2);
MotorControl turretMotor(TURRET_IN1, TURRET_IN2);
ServoControl steeringServo(STEERING_SERVO_PIN);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
GPIOViewer gpio_viewer;

int currentDriveSpeed = 0;
int targetDriveSpeed = 0;
int turretCurrentSpeed = 0;
int turretTargetSpeed = 0;

bool turretActive = false;
uint32_t lastWsActivity = 0;

// ---------- Drive ----------
void driveForward() {
  if (currentDriveSpeed == 0)
    currentDriveSpeed = DRIVE_MIN_SPEED;
  targetDriveSpeed = DRIVE_MAX_SPEED;
  Serial.println("Drive -> Forward");
}

void driveReverse() {
  if (currentDriveSpeed == 0)
    currentDriveSpeed = -DRIVE_MIN_SPEED;
  targetDriveSpeed = -DRIVE_MAX_SPEED;
  Serial.println("Drive -> Reverse");
}

void driveStop() {
  targetDriveSpeed = 0;
  Serial.println("Drive -> Stop");
}

void updateDriveMotor() {
  if (currentDriveSpeed < targetDriveSpeed) {
    currentDriveSpeed += 3 * DRIVE_STEP;
    if (currentDriveSpeed > targetDriveSpeed)
      currentDriveSpeed = targetDriveSpeed;
  } else if (currentDriveSpeed > targetDriveSpeed) {
    currentDriveSpeed -= DRIVE_STEP;
    if (currentDriveSpeed < targetDriveSpeed)
      currentDriveSpeed = targetDriveSpeed;
  }

  driveMotor.setSpeed(currentDriveSpeed);
}

// ---------- Steering ----------
void steerLeft() {
  steeringServo.setAngle(LEFT_ANGLE);
}

void steerCenter() {
  steeringServo.setAngle(CENTER_ANGLE);
}

void steerRight() {
  steeringServo.setAngle(RIGHT_ANGLE);
}

// ---------- Turret ----------
void turretLeft() {
  turretActive = true;

  if (turretCurrentSpeed == 0)
    turretCurrentSpeed = TURRET_MIN_SPEED;

  turretTargetSpeed = TURRET_MAX_SPEED;
  Serial.println("Turret -> Left");
}

void turretRight() {
  turretActive = true;

  if (turretCurrentSpeed == 0)
    turretCurrentSpeed = -TURRET_MIN_SPEED;

  turretTargetSpeed = -TURRET_MAX_SPEED;
  Serial.println("Turret -> Right");
}

void turretStop() {
  turretActive = false;
  turretTargetSpeed = 0;
  Serial.println("Turret -> Stop");
}

void updateTurretMotor() {
  if (turretCurrentSpeed < turretTargetSpeed) {
    turretCurrentSpeed += TURRET_STEP;
    if (turretCurrentSpeed > turretTargetSpeed)
      turretCurrentSpeed = turretTargetSpeed;
  } else if (turretCurrentSpeed > turretTargetSpeed) {
    turretCurrentSpeed -= TURRET_STEP;
    if (turretCurrentSpeed < turretTargetSpeed)
      turretCurrentSpeed = turretTargetSpeed;
  }

  turretMotor.setSpeed(turretCurrentSpeed);

  if (!turretActive && turretCurrentSpeed == 0)
    turretMotor.coast();
}

// ---------- Safety ----------
void stopAll() {
  driveStop();
  turretStop();
  steerCenter();
}

// ---------- WebSocket ----------
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
      lastWsActivity = millis();
      Serial.printf("Client #%u connected\n", client->id());
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("Client #%u disconnected\n", client->id());
      stopAll();
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;

      if (!(info->final &&
            info->index == 0 &&
            info->len == len &&
            info->opcode == WS_TEXT))
        break;

      lastWsActivity = millis();

      String cmd;
      cmd.reserve(len);
      for (size_t i = 0; i < len; i++)
        cmd += (char)data[i];

      if (cmd == "system:stop" || cmd == "system:stopall") {
        stopAll();
        break;
      }

      int separator = cmd.indexOf(':');
      if (separator < 0)
        break;

      String group = cmd.substring(0, separator);
      String action = cmd.substring(separator + 1);

      if (group == "drive") {
        if (action == "forward")
          driveForward();
        else if (action == "reverse")
          driveReverse();
        else if (action == "stop")
          driveStop();
      } else if (group == "steer") {
        if (action == "left")
          steerLeft();
        else if (action == "right")
          steerRight();
        else if (action == "center")
          steerCenter();
      } else if (group == "turret") {
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

// ---------- WiFi / Web Server ----------
void initWiFi() {
  Serial.print("Connecting to WiFi");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());
}

void initWebServer() {
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->redirect("/main.html");
  });

  server.serveStatic("/", LittleFS, "/");

  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "404 - File Not Found");
  });

  server.begin();

  Serial.println();
  Serial.println("===============================");
  Serial.println("||  Material Handler Ready   ||");
  Serial.println("===============================");
  Serial.print("||  IP Address : ");
  Serial.print(WiFi.localIP());
  Serial.println("  ||");
  Serial.println("==============================");
}

// ---------- Setup / Loop ----------
void initHardware() {
  pinMode(MOTOR_SLEEP_PIN, OUTPUT);
  digitalWrite(MOTOR_SLEEP_PIN, LOW);

  driveMotor.begin();
  turretMotor.begin();
  steeringServo.begin(CENTER_ANGLE);
}

void setup() {
  Serial.begin(115200);

  initHardware();

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed.");
    while (true)
      delay(1000);
  }

  initWiFi();
  initWebServer();
  gpio_viewer.begin();

  lastWsActivity = millis();
}

void loop() {
  ws.cleanupClients();

  if (ws.count() == 0) {
    stopAll();
  } else if (millis() - lastWsActivity > WS_TIMEOUT_MS) {
    stopAll();
  }

  updateDriveMotor();
  updateTurretMotor();

  bool motorsActive =
    currentDriveSpeed != 0 ||
    targetDriveSpeed != 0 ||
    turretCurrentSpeed != 0 ||
    turretTargetSpeed != 0;

  digitalWrite(MOTOR_SLEEP_PIN, motorsActive ? HIGH : LOW);
}
