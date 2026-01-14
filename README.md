# xa-b-ng-xe-t-h-nh-
#include "esp_camera.h"
 #include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <algorithm>

// Motor pins
#define ENA 12
#define IN1 13
#define IN2 15
#define IN3 14
#define IN4 2

#define LIGHT_PIN 4

#define TRIG_PIN 4
#define ECHO_PIN 16

#define FORWARD 1
#define BACKWARD -1
#define STOP 0

#define UP 1
  #define DOWN 2
#define LEFT 3
#define RIGHT 4
#define STOP_CMD 0

const int PWMFreq = 1000;
const int PWMResolution = 8;
const int PWMSpeedChannel = 0;
const int PWMLightChannel = 1;

enum AutoState { MOVING_FORWARD, BACKING_UP, TURNING_LEFT, TURNING_RIGHT, STOPPED };
AutoState autoState = STOPPED;

AsyncWebServer server(80);
AsyncWebSocket wsCamera("/Camera");
AsyncWebSocket wsCarInput("/CarInput");

uint32_t cameraClientId = 0;

volatile bool autoMode = false;
volatile int motorSpeed = 150;
const int MIN_SPEED = 80;

// Camera pin config ESP32-CAM AI-Thinker
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char* ssid     = "Cam_Car nhom 10";
const char* password = "12345678";

const char* htmlHomePage PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>ESP32-CAM Car Control</title>
  <style>
    body {background:#f9e9f9; font-family: Arial;}
    #cameraImage {width: 100%; max-width: 400px; height: auto; border: 1px solid #ccc;}
    .button {font-size: 40px; background: white; border-radius: 80px; padding: 10px; margin: 50px; box-shadow: 2px 2px 5px #999;}
    .button:active {transform: translate(2px,2px); box-shadow: none;}
    #controls {text-align: center; margin-top: 10px;}
    #speedSlider, #lightSlider {width: 90%;}
    label {font-weight: bold;}
    #autoBtn {background: #28a745; color: white; font-size: 18px; padding: 10px; border:none; border-radius: 8px; cursor: pointer; margin-top: 10px;}
    #autoBtn.off {background: #dc3545;}
  </style>
</head>
<body>
  <h2 style="text-align:center;">ESP32-CAM Car Control</h2>
  <img id="cameraImage" src="" alt="Camera Stream"/>

  <div id="controls">
    <div>
      <button class="button" ontouchstart="sendMove(1)" ontouchend="sendMove(0)">&#8679;</button>
    </div>
    <div>
      <button class="button" ontouchstart="sendMove(3)" ontouchend="sendMove(0)">&#8678;</button>
      <button class="button" ontouchstart="sendMove(4)" ontouchend="sendMove(0)">&#8680;</button>
    </div>
    <div>
      <button class="button" ontouchstart="sendMove(2)" ontouchend="sendMove(0)">&#8681;</button>
    </div>
    <div>
      <label for="speedSlider">Speed:</label><br>
      <input type="range" id="speedSlider" min="0" max="255" value="150" oninput="sendSpeed(this.value)" onchange="sendSpeed(this.value)">
    </div>
    <div>
      <label for="lightSlider">Light:</label><br>
      <input type="range" id="lightSlider" min="0" max="255" value="0" oninput="sendLight(this.value)" onchange="sendLight(this.value)">
    </div>
    <div>
      <button id="autoBtn" onclick="toggleAuto()">Auto Mode OFF</button>
    </div>
  </div>

<script>
  var wsCamera = new WebSocket('ws://' + location.hostname + '/Camera');
  wsCamera.binaryType = 'blob';
  wsCamera.onmessage = function(event) {
    var img = document.getElementById('cameraImage');
    img.src = URL.createObjectURL(event.data);
  };
  wsCamera.onclose = function() { setTimeout(() => location.reload(), 2000); };

  var wsCarInput = new WebSocket('ws://' + location.hostname + '/CarInput');
  wsCarInput.onopen = function() {
    sendSpeed(document.getElementById('speedSlider').value);
    sendLight(document.getElementById('lightSlider').value);
  };
  wsCarInput.onclose = function() { setTimeout(() => location.reload(), 2000); };

  function sendMove(val) {
    if (autoMode) return;
    wsCarInput.send('MoveCar,' + val);
  }

  function sendSpeed(val) {
    motorSpeed = val;
    wsCarInput.send('Speed,' + val);
    document.getElementById('speedSlider').value = val;
  }

  function sendLight(val) {
    wsCarInput.send('Light,' + val);
    document.getElementById('lightSlider').value = val;
  }

  var autoMode = false;
  function toggleAuto() {
    autoMode = !autoMode;
    wsCarInput.send('AutoMode,' + (autoMode ? '1' : '0'));
    wsCarInput.send('Speed,' + motorSpeed);
    var btn = document.getElementById('autoBtn');
    btn.textContent = 'Auto Mode ' + (autoMode ? 'ON' : 'OFF');
    btn.className = autoMode ? 'on' : 'off';
  }
</script>
</body>
</html>
)rawliteral";

void setMotor(int motor, int direction) {
  if (motor == 0) {
    if (direction == FORWARD) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else if (direction == BACKWARD) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
  } else {
    if (direction == FORWARD) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if (direction == BACKWARD) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
  }
}

void moveCar(int cmd) {
  switch(cmd) {
    case UP:
      setMotor(0, FORWARD);
      setMotor(1, FORWARD);
      autoState = MOVING_FORWARD;
      break;
    case DOWN:
      setMotor(0, BACKWARD);
      setMotor(1, BACKWARD);
      autoState = BACKING_UP;
      break;
    case LEFT:
      setMotor(0, BACKWARD);
      setMotor(1, FORWARD);
      autoState = TURNING_LEFT;
      break;
    case RIGHT:
      setMotor(0, FORWARD);
      setMotor(1, BACKWARD);
      autoState = TURNING_RIGHT;
      break;
    case STOP_CMD:
    default:
      setMotor(0, STOP);
      setMotor(1, STOP);
      autoState = STOPPED;
      break;
  }
  ledcWrite(PWMSpeedChannel, motorSpeed);
}

long readUltrasonicDistance() {
  const int numReadings = 3;
  long readings[numReadings];
  long total = 0;
  int validReadings = 0;

  for (int i = 0; i < numReadings; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

 long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration > 0) {
      long distance = duration * 0.034 / 2;
      if (distance > 0 && distance < 400) {
        readings[validReadings] = distance;
        validReadings++;
      }
    }
    delay(10);
  }

  if (validReadings == 0) return -1;

  for (int i = 0; i < validReadings; i++) {
    total += readings[i];
  }
  return total / validReadings;
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
}

void sendCameraPicture() {
  if (cameraClientId == 0) return;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  wsCamera.binary(cameraClientId, fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void onCameraWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                            void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("Camera WS client #%u connected\n", client->id());
      cameraClientId = client->id();
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("Camera WS client #%u disconnected\n", client->id());
      if (cameraClientId == client->id()) cameraClientId = 0;
      break;
    default:
      break;
  }
}

void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                             void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String msg = (const char*)data;
      int commaIndex = msg.indexOf(',');
      if (commaIndex > 0) {
        String key = msg.substring(0, commaIndex);
        String val = msg.substring(commaIndex + 1);
        Serial.printf("Key: %s, Value: %s\n", key.c_str(), val.c_str());

  if (key == "MoveCar") {
          if (!autoMode) {
            int v = val.toInt();
            moveCar(v);
          }
        } else if (key == "Speed") {
          motorSpeed = std::max((int)val.toInt(), MIN_SPEED);
          ledcWrite(PWMSpeedChannel, motorSpeed);
          Serial.printf("Updated motorSpeed: %d\n", motorSpeed);
        } else if (key == "Light") {
          int lightVal = val.toInt();
          ledcWrite(PWMLightChannel, lightVal);
        } else if (key == "AutoMode") {
          autoMode = (val.toInt() == 1);
          if (!autoMode) {
            moveCar(STOP_CMD);
            autoState = STOPPED;
          } else {
            ledcWrite(PWMSpeedChannel, motorSpeed);
          }
        }
      }
    }
  } else if (type == WS_EVT_CONNECT) {
    Serial.println("CarInput WS client connected");
    wsCarInput.text(client->id(), "Speed," + String(motorSpeed));
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("CarInput WS client disconnected");
    moveCar(STOP_CMD);
    ledcWrite(PWMLightChannel, 0);
    autoState = STOPPED;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  ledcSetup(PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(ENA, PWMSpeedChannel);

  ledcSetup(PWMLightChannel, PWMFreq, PWMResolution);
  ledcAttachPin(LIGHT_PIN, PWMLightChannel);

  ledcWrite(PWMSpeedChannel, motorSpeed);
  ledcWrite(PWMLightChannel, 0);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", htmlHomePage);
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
  });

  wsCamera.onEvent(onCameraWebSocketEvent);
  server.addHandler(&wsCamera);

  wsCarInput.onEvent(onCarInputWebSocketEvent);
  server.addHandler(&wsCarInput);

  server.begin();
  Serial.println("HTTP server started");

  setupCamera();
}

unsigned long lastAutoCheck = 0;
const unsigned long autoCheckInterval = 100;
unsigned long lastActionTime = 0;
const unsigned long backUpDuration = 500; // Thời gian lùi (ms)
const unsigned long turnDuration = 400;   // Thời gian rẽ (ms)

void loop() {
  wsCamera.cleanupClients();
  wsCarInput.cleanupClients();

  sendCameraPicture();

  if (autoMode) {
    unsigned long now = millis();
    if (now - lastAutoCheck > autoCheckInterval) {
      lastAutoCheck = now;
      long dist = readUltrasonicDistance();
      Serial.printf("Distance: %ld cm, State: %d, Speed: %d\n", dist, autoState, motorSpeed);

   if (dist > 0 && dist < 1000) {
        if (autoState != BACKING_UP && autoState != TURNING_LEFT && autoState != TURNING_RIGHT) {
          moveCar(DOWN); // Lùi lại
          lastActionTime = now;
          autoState = BACKING_UP; 
        }
      } else if (autoState == BACKING_UP && now - lastActionTime > backUpDuration) {
        // Chọn ngẫu nhiên rẽ trái hoặc phải
        int turnDirection = random(0, 2) == 0 ? LEFT : RIGHT;
        moveCar(turnDirection);
        lastActionTime = now;
        autoState = (turnDirection == LEFT) ? TURNING_LEFT : TURNING_RIGHT;
      } else if ((autoState == TURNING_LEFT || autoState == TURNING_RIGHT) && now - lastActionTime > turnDuration) {
        moveCar(UP); // Tiếp tục tiến lên
        autoState = MOVING_FORWARD;
      } else if (autoState == STOPPED || autoState == MOVING_FORWARD) {
        moveCar(UP);
        autoState = MOVING_FORWARD;
      }
      ledcWrite(PWMSpeedChannel, motorSpeed);
    }
  }

  delay(5);
}


