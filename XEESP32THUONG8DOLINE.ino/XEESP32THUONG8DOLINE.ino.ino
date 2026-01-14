#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ====== MOTOR PINS (2 PWM riêng) ======
#define ENA_LEFT  12   // PWM trái
#define IN1       13
#define IN2       15
#define IN3       14
#define IN4       2
#define ENB_RIGHT 5    // PWM phải

// ====== LIGHT & ULTRASONIC ======
#define LIGHT_PIN 4
#define TRIG_PIN  18
#define ECHO_PIN  23

// ====== LINE SENSORS ======
#define S1 35
#define S2 32
#define S3 33
#define S4 25
#define S5 26

// ====== CONSTANTS ======
#define FORWARD  1
#define BACKWARD -1
#define STOP     0
#define UP    1
#define DOWN  2
#define LEFT  3
#define RIGHT 4
#define STOP_CMD 0

const int PWMFreq = 1000;
const int PWMResolution = 8;

const int PWM_LEFT_CHANNEL  = 0;
const int PWM_RIGHT_CHANNEL = 1;
const int PWM_LIGHT_CHANNEL = 2;

volatile int baseSpeed = 150;
volatile int leftSpeed = 150;
volatile int rightSpeed = 150;
const int MIN_SPEED = 80;
const int MAX_SPEED = 255;

AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");

volatile bool autoMode = false;
volatile bool lineMode = false;

const char* ssid = "Cam_Car_nhom10";
const char* password = "12345678";

// ====== PID FOR LINE FOLLOWING ======
float kp = 35.0;
float kd = 15.0;
int lastError = 0;

// ====== AUTO STATE MACHINE ======
enum AutoState {
  MOVING_FORWARD,
  BACKING_UP,
  TURNING,
  CHECKING
};

// ====== GIAO DIỆN WEB SIÊU ĐẸP (CẬP NHẬT NHẸ) ======
const char* htmlHomePage PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>ESP32 Car Control</title>
  <style>
    :root {
      --primary: #6c5ce7;
      --success: #00b894;
      --danger: #d63031;
      --warning: #fdcb6e;
      --dark: #2d3436;
      --light: #f8f9fa;
      --gray: #b2bec3;
    }
    * { margin:0; padding:0; box-sizing:border-box; }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      color: white;
      min-height: 100vh;
      padding: 15px;
    }
    .container {
      max-width: 500px;
      margin: 0 auto;
      background: rgba(255,255,255,0.1);
      backdrop-filter: blur(10px);
      border-radius: 20px;
      padding: 20px;
      box-shadow: 0 15px 35px rgba(0,0,0,0.2);
    }
    h1 {
      text-align: center;
      font-size: 28px;
      margin-bottom: 20px;
      text-shadow: 0 2px 5px rgba(0,0,0,0.3);
    }
    .status-bar {
      display: flex;
      justify-content: space-around;
      margin-bottom: 20px;
      font-weight: bold;
    }
    .status-item {
      text-align: center;
      padding: 10px;
      border-radius: 12px;
      background: rgba(255,255,255,0.15);
      flex: 1;
      margin: 0 5px;
      font-size: 14px;
    }
    .status-item span { display: block; font-size: 18px; margin-top: 5px; }

    .joypad {
      position: relative;
      width: 200px;
      height: 200px;
      margin: 30px auto;
      background: rgba(255,255,255,0.1);
      border-radius: 50%;
      border: 4px solid rgba(255,255,255,0.3);
      overflow: hidden;
    }
    .joypad::before {
      content: '';
      position: absolute;
      top: 50%; left: 50%;
      width: 60px; height: 60px;
      background: var(--primary);
      border-radius: 50%;
      transform: translate(-50%, -50%);
      box-shadow: 0 0 20px rgba(108,92,231,0.6);
      transition: all 0.1s;
    }
    .joypad:active::before { transform: translate(-50%, -50%) scale(1.2); }

    .dir-btn {
      position: absolute;
      width: 50%; height: 50%;
      cursor: pointer;
      z-index: 1;
    }
    #up { top: 0; left: 25%; width: 50%; height: 50%; }
    #down { bottom: 0; left: 25%; width: 50%; height: 50%; }
    #left { top: 25%; left: 0; width: 50%; height: 50%; }
    #right { top: 25%; right: 0; width: 50%; height: 50%; }

    .control-group {
      margin: 20px 0;
      padding: 15px;
      background: rgba(255,255,255,0.1);
      border-radius: 15px;
    }
    label {
      display: block;
      margin-bottom: 8px;
      font-weight: 600;
      font-size: 16px;
    }
    .slider-container {
      display: flex;
      align-items: center;
      gap: 10px;
    }
    input[type="range"] {
      flex: 1;
      height: 8px;
      border-radius: 5px;
      background: rgba(255,255,255,0.3);
      outline: none;
      -webkit-appearance: none;
    }
    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 20px; height: 20px;
      background: var(--primary);
      border-radius: 50%;
      cursor: pointer;
      box-shadow: 0 0 10px rgba(108,92,231,0.6);
    }
    .value {
      min-width: 50px;
      text-align: center;
      font-weight: bold;
      font-size: 18px;
    }

    .mode-buttons {
      display: flex;
      justify-content: center;
      gap: 15px;
      margin: 25px 0;
    }
    .mode-btn {
      flex: 1;
      padding: 14px;
      border: none;
      border-radius: 15px;
      font-weight: bold;
      font-size: 16px;
      cursor: pointer;
      transition: all 0.3s;
      position: relative;
      overflow: hidden;
    }
    .mode-btn::before {
      content: '';
      position: absolute;
      top: 0; left: -100%;
      width: 100%; height: 100%;
      background: linear-gradient(90deg, transparent, rgba(255,255,255,0.3), transparent);
      transition: 0.5s;
    }
    .mode-btn:hover::before { left: 100%; }
    #autoBtn { background: var(--danger); }
    #autoBtn.on { background: var(--success); }
    #lineBtn { background: var(--gray); }
    #lineBtn.on { background: #0984e3; }

    #status {
      text-align: center;
      padding: 12px;
      background: rgba(0,0,0,0.2);
      border-radius: 12px;
      margin-top: 15px;
      font-weight: bold;
      min-height: 20px;
      word-wrap: break-word;
    }

    @media (max-width: 480px) {
      .container { padding: 15px; }
      h1 { font-size: 24px; }
      .joypad { width: 180px; height: 180px; }
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>ESP32 Car Control</h1>

    <div class="status-bar">
      <div class="status-item">
        Mode<br>
        <span id="modeStatus">Manual</span>
      </div>
      <div class="status-item">
        Khoảng cách<br>
        <span id="distStatus">-- cm</span>
      </div>
      <div class="status-item">
        Tốc độ<br>
        <span id="speedVal">150</span>
      </div>
    </div>

    <div class="joypad" id="joypad">
      <div class="dir-btn" id="up"></div>
      <div class="dir-btn" id="down"></div>
      <div class="dir-btn" id="left"></div>
      <div class="dir-btn" id="right"></div>
    </div>

    <div class="control-group">
      <label>Tốc độ điều khiển</label>
      <div class="slider-container">
        <input type="range" id="speedSlider" min="80" max="255" value="150">
        <div class="value" id="speedValue">150</div>
      </div>
    </div>

    <div class="control-group">
      <label>Độ sáng đèn</label>
      <div class="slider-container">
        <input type="range" id="lightSlider" min="0" max="255" value="0">
        <div class="value" id="lightValue">0</div>
      </div>
    </div>

    <div class="mode-buttons">
      <button id="autoBtn" class="mode-btn off">Auto OFF</button>
      <button id="lineBtn" class="mode-btn off">Line OFF</button>
    </div>

    <div id="status">Đang kết nối...</div>
  </div>

<script>
  const ws = new WebSocket('ws://' + location.hostname + '/CarInput');
  const statusEl = document.getElementById('status');
  const modeStatus = document.getElementById('modeStatus');
  const distStatus = document.getElementById('distStatus');

  ws.onopen = () => statusEl.textContent = 'Đã kết nối!';
  ws.onclose = () => statusEl.textContent = 'Mất kết nối!';
  ws.onmessage = (e) => {
    const msg = e.data;
    
    // Cập nhật trạng thái hành động
    if (msg.includes('Phát hiện') || msg.includes('Rẽ') || msg.includes('Đường thông') || msg.includes('Vẫn gần')) {
      statusEl.textContent = msg;
    }

    if (msg.includes('AutoMode')) {
      const on = msg.includes('ON');
      document.getElementById('autoBtn').textContent = 'Auto ' + (on?'ON':'OFF');
      document.getElementById('autoBtn').className = 'mode-btn ' + (on?'on':'off');
      modeStatus.textContent = on ? 'Auto' : (document.getElementById('lineBtn').classList.contains('on') ? 'Line' : 'Manual');
    }
    if (msg.includes('LineMode')) {
      const on = msg.includes('ON');
      document.getElementById('lineBtn').textContent = 'Line ' + (on?'ON':'OFF');
      document.getElementById('lineBtn').className = 'mode-btn ' + (on?'on':'off');
      modeStatus.textContent = on ? 'Line' : (document.getElementById('autoBtn').classList.contains('on') ? 'Auto' : 'Manual');
    }
    if (msg.includes('Dist:')) {
      distStatus.textContent = msg.split(':')[1].trim();
    }
  };

  const joypad = document.getElementById('joypad');
  const dirs = { up: false, down: false, left: false, right: false };
  let sendTimer;

  function sendDirection() {
    let cmd = 0;
    if (dirs.up) cmd = 1;
    else if (dirs.down) cmd = 2;
    else if (dirs.left) cmd = 3;
    else if (dirs.right) cmd = 4;
    ws.send('M,' + cmd);
  }

  function startSend() {
    if (!sendTimer) {
      sendTimer = setInterval(sendDirection, 100);
    }
  }
  function stopSend() {
    clearInterval(sendTimer);
    sendTimer = null;
    ws.send('M,0');
  }

  ['up','down','left','right'].forEach(dir => {
    const el = document.getElementById(dir);
    el.addEventListener('mousedown', () => { dirs[dir] = true; startSend(); });
    el.addEventListener('mouseup', () => { dirs[dir] = false; if (!Object.values(dirs).some(v=>v)) stopSend(); });
    el.addEventListener('touchstart', (e) => { e.preventDefault(); dirs[dir] = true; startSend(); });
    el.addEventListener('touchend', () => { dirs[dir] = false; if (!Object.values(dirs).some(v=>v)) stopSend(); });
  });

  document.getElementById('speedSlider').oninput = function() {
    const val = this.value;
    document.getElementById('speedValue').textContent = val;
    document.getElementById('speedVal').textContent = val;
    ws.send('S,' + val);
  };
  document.getElementById('lightSlider').oninput = function() {
    const val = this.value;
    document.getElementById('lightValue').textContent = val;
    ws.send('L,' + val);
  };

  document.getElementById('autoBtn').onclick = () => {
    const btn = document.getElementById('autoBtn');
    const turnOn = btn.classList.contains('off');
    ws.send('A,' + (turnOn?'1':'0'));
  };
  document.getElementById('lineBtn').onclick = () => {
    const btn = document.getElementById('lineBtn');
    const turnOn = btn.classList.contains('off');
    ws.send('B,' + (turnOn?'1':'0'));
  };

  document.getElementById('speedValue').textContent = 150;
  document.getElementById('speedVal').textContent = 150;
</script>
</body>
</html>
)rawliteral";

// ====== MOTOR CONTROL ======
void setMotorDirection(int motor, int dir) {
  if (motor == 0) {
    digitalWrite(IN1, dir == FORWARD);
    digitalWrite(IN2, dir == BACKWARD);
  } else {
    digitalWrite(IN3, dir == FORWARD);
    digitalWrite(IN4, dir == BACKWARD);
  }
}

void setMotorSpeed(int left, int right) {
  leftSpeed = constrain(left, 0, MAX_SPEED);
  rightSpeed = constrain(right, 0, MAX_SPEED);
  ledcWrite(PWM_LEFT_CHANNEL, leftSpeed);
  ledcWrite(PWM_RIGHT_CHANNEL, rightSpeed);
}

void moveCar(int cmd) {
  switch (cmd) {
    case UP:    setMotorDirection(0, FORWARD);  setMotorDirection(1, FORWARD);  setMotorSpeed(baseSpeed, baseSpeed); break;
    case DOWN:  setMotorDirection(0, BACKWARD); setMotorDirection(1, BACKWARD); setMotorSpeed(baseSpeed, baseSpeed); break;
    case LEFT:  setMotorDirection(0, BACKWARD); setMotorDirection(1, FORWARD);  setMotorSpeed(baseSpeed, baseSpeed); break;
    case RIGHT: setMotorDirection(0, FORWARD);  setMotorDirection(1, BACKWARD); setMotorSpeed(baseSpeed, baseSpeed); break;
    case STOP_CMD:
    default:    setMotorDirection(0, STOP);     setMotorDirection(1, STOP);     setMotorSpeed(0, 0); break;
  }
}

void stopCar() {
  setMotorSpeed(0, 0);
}

// ====== ULTRASONIC ======
long readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;

  long distance = duration * 0.034 / 2;
  return constrain(distance, 0, 400);
}

// ====== LINE FOLLOWING PID ======
int readLinePosition() {
  int s[5] = {!digitalRead(S1), !digitalRead(S2), !digitalRead(S3), !digitalRead(S4), !digitalRead(S5)};
  int sum = s[0] + s[1] + s[2] + s[3] + s[4];
  if (sum == 0) return lastError;
  int weighted = (s[0]*(-200) + s[1]*(-100) + s[2]*0 + s[3]*100 + s[4]*200);
  return weighted / sum;
}

void doLineFollowing() {
  int error = readLinePosition();
  int derivative = error - lastError;
  int correction = (kp * error) + (kd * derivative);

  int left = baseSpeed + correction;
  int right = baseSpeed - correction;

  left = constrain(left, MIN_SPEED, MAX_SPEED);
  right = constrain(right, MIN_SPEED, MAX_SPEED);

  setMotorDirection(0, FORWARD);
  setMotorDirection(1, FORWARD);
  setMotorSpeed(left, right);

  lastError = error;
}

// ====== AUTO AVOID OBSTACLE - CẢI TIẾN VỚI STATE MACHINE ======
void doAutoMode() {
  static AutoState state = MOVING_FORWARD;
  static unsigned long stateStartTime = 0;
  static int turnDirection = 0; // -1: left, 1: right

  long dist = readUltrasonicDistance();
  int obstacleThreshold = (baseSpeed > 200) ? 30 : 25; // Tốc độ cao → phát hiện sớm hơn

  // GỬI KHOẢNG CÁCH REALTIME
  static unsigned long lastDistSend = 0;
  if (millis() - lastDistSend > 200) {
    wsCarInput.textAll("Dist: " + String(dist) + " cm");
    lastDistSend = millis();
  }

  // === STATE MACHINE ===
  switch (state) {
    case MOVING_FORWARD:
      if (dist < obstacleThreshold) {
        setMotorDirection(0, BACKWARD);
        setMotorDirection(1, BACKWARD);
        setMotorSpeed(180, 180);
        state = BACKING_UP;
        stateStartTime = millis();
        wsCarInput.textAll("Phát hiện vật cản! Lùi lại...");
      } else {
        setMotorDirection(0, FORWARD);
        setMotorDirection(1, FORWARD);
        setMotorSpeed(baseSpeed, baseSpeed);
      }
      break;

    case BACKING_UP:
      if (millis() - stateStartTime > 600) {
        turnDirection = random(0, 2) ? 1 : -1;
        if (turnDirection == 1) {
          setMotorDirection(0, FORWARD);  setMotorDirection(1, BACKWARD);
          wsCarInput.textAll("Rẽ phải tránh vật cản");
        } else {
          setMotorDirection(0, BACKWARD); setMotorDirection(1, FORWARD);
          wsCarInput.textAll("Rẽ trái tránh vật cản");
        }
        setMotorSpeed(230, 230);
        state = TURNING;
        stateStartTime = millis();
      }
      break;

    case TURNING:
      if (millis() - stateStartTime > 1000) {
        state = CHECKING;
        stateStartTime = millis();
        stopCar();
        wsCarInput.textAll("Kiểm tra lại phía trước...");
      }
      break;

    case CHECKING:
      if (millis() - stateStartTime > 300) {
        long newDist = readUltrasonicDistance();
        if (newDist >= obstacleThreshold) {
          state = MOVING_FORWARD;
          wsCarInput.textAll("Đường thông! Tiếp tục tiến");
        } else {
          wsCarInput.textAll("Vẫn gần! Lùi thêm & đổi hướng");
          turnDirection = -turnDirection;
          setMotorDirection(0, BACKWARD);
          setMotorDirection(1, BACKWARD);
          setMotorSpeed(180, 180);
          state = BACKING_UP;
          stateStartTime = millis();
        }
      }
      break;
  }
}

// ====== WEBSOCKET HANDLER ======
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    String msg = String((char*)data).substring(0, len);
    int comma = msg.indexOf(',');
    if (comma <= 0) {
      if (msg == "0") { stopCar(); client->text("STOP"); }
      return;
    }
    String key = msg.substring(0, comma);
    String val = msg.substring(comma + 1);

    if (key == "M" && !autoMode && !lineMode) {
      moveCar(val.toInt());
    }
    else if (key == "S") {
      baseSpeed = constrain(val.toInt(), MIN_SPEED, MAX_SPEED);
      setMotorSpeed(baseSpeed, baseSpeed);
    }
    else if (key == "L") {
      ledcWrite(PWM_LIGHT_CHANNEL, val.toInt());
    }
    else if (key == "A") {
      autoMode = (val == "1");
      lineMode = false;
      stopCar();
      client->text("AutoMode," + String(autoMode ? "ON" : "OFF"));
    }
    else if (key == "B") {
      lineMode = (val == "1");
      autoMode = false;
      stopCar();
      lastError = 0;
      client->text("LineMode," + String(lineMode ? "ON" : "OFF"));
    }
  }
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA_LEFT, OUTPUT); pinMode(ENB_RIGHT, OUTPUT);

  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLUP);
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(S3, INPUT_PULLUP);
  pinMode(S4, INPUT_PULLUP);
  pinMode(S5, INPUT_PULLUP);

  ledcSetup(PWM_LEFT_CHANNEL, PWMFreq, PWMResolution);
  ledcSetup(PWM_RIGHT_CHANNEL, PWMFreq, PWMResolution);
  ledcSetup(PWM_LIGHT_CHANNEL, PWMFreq, PWMResolution);
  ledcAttachPin(ENA_LEFT, PWM_LEFT_CHANNEL);
  ledcAttachPin(ENB_RIGHT, PWM_RIGHT_CHANNEL);
  ledcAttachPin(LIGHT_PIN, PWM_LIGHT_CHANNEL);

  randomSeed(analogRead(0) + micros()); // Tốt hơn

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: http://"); Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", htmlHomePage);
  });
  wsCarInput.onEvent(onWsEvent);
  server.addHandler(&wsCarInput);
  server.begin();

  stopCar();
  Serial.println("Xe sẵn sàng! Auto Mode siêu thông minh!");
}

// ====== LOOP ======
unsigned long lastUpdate = 0;
void loop() {
  wsCarInput.cleanupClients();

  if (autoMode) {
    doAutoMode();
  }
  else if (lineMode) {
    if (millis() - lastUpdate > 20) {
      doLineFollowing();
      lastUpdate = millis();
    }
  }

  delay(5);
}