#include <WiFi.h>
#include "VideoStream.h"
#include "StreamIO.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"
#include <AmebaServo.h>

#define CHANNEL     0      // MJPEG & OSD 通道
#define CHANNEL_NN  3      // RGB for NN
#define FIRE_LED    18     // FIRE LED 腳位

// MJPEG 設定：640×480 @15fps, JPEG 編碼, buffer 1
VideoSetting configJPG(640, 480, 15, VIDEO_JPEG, 1);
// NN 偵測通道設定：576×320 @10fps, RGB
VideoSetting configNN(576, 320, 10, VIDEO_RGB, 0);

NNObjectDetection ObjDet;
StreamIO           videoStreamerNN(1, 1);
WiFiServer         server(80);

AmebaServo servoPan;   // 左右
AmebaServo servoTilt;  // 上下

int panAngle  = 90;
int tiltAngle = 90;

char ssid[] = "********";
char pass[] = "********";
uint32_t img_addr = 0;
uint32_t img_len  = 0;
// 模式切換旗標
bool aimingMode = false;  // false: 手動, true: 自動
bool fireEnabled = true;  // 開火權限

// 相機垂直視野與影像高度
const float V_FOV = 45.0;
const int   IMG_H = 480;

// ————————————————————————————
// 1) 物件偵測結果 callback：做 OSD 標註（僅自動模式）
// ————————————————————————————
void ODPostProcess(std::vector<ObjectDetectionResult> results) {
  // 清空畫面
  OSD.createBitmap(CHANNEL);
  OSD.update(CHANNEL);
  if (!aimingMode) return;

  uint16_t w = configJPG.width();
  uint16_t h = configJPG.height();
  // 畫出所有目標框
  for (auto &item : results) {
    int x1 = item.xMin() * w;
    int y1 = item.yMin() * h;
    int x2 = item.xMax() * w;
    int y2 = item.yMax() * h;
    OSD.drawRect(CHANNEL, x1, y1, x2, y2, 2, OSD_COLOR_WHITE);
    char buf[32];
    snprintf(buf, sizeof(buf), "%s %d",
             itemList[item.type()].objectName, item.score());
    OSD.drawText(CHANNEL, x1, y1 - 10, buf, OSD_COLOR_CYAN);
  }

  // 篩選特定類別 (class index == 1) 做自動瞄準
  const int deadZonePx = 20;  // 死區 (像素)
  for (auto &item : results) {
    if (item.type() != 1) continue;
    int x1 = item.xMin() * w;
    int y1 = item.yMin() * h;
    int x2 = item.xMax() * w;
    int y2 = item.yMax() * h;
    int cx = (x1 + x2) / 2;
    int cy = (y1 + y2) / 2;

    // 垂直偏移 (像素)
    int dy = cy - (h / 2);
    // 只有超過死區才執行
    if (abs(dy) > deadZonePx) {
      float degPerPix = V_FOV / IMG_H;
      float deltaAng  = dy * degPerPix;
      tiltAngle = constrain(90.0f - deltaAng, 0.0f, 180.0f);
      servoTilt.write(tiltAngle);
    }

    // 水平偏移 (像素)
    int dx = cx - (w / 2);
    if (abs(dx) > deadZonePx) {
      float H_FOV = 60.0;
      float degPerPixX = H_FOV / w;
      float deltaYaw = dx * degPerPixX;
      panAngle = constrain(90.0f + deltaYaw, 0.0f, 180.0f);
      servoPan.write(panAngle);
    }

    break; // 只鎖定第一筆
  }

  // 偵測到類別 0 時鎖死開火
  fireEnabled = true;
  for (auto &item : results) {
    if (item.type() == 0) {
      fireEnabled = false;
      break;
    }
  }

  OSD.update(CHANNEL);
}

// ... 其餘程式不變 ...


// ... 其餘程式不變 ...


// ————————————————————————————
// 2) HTTP: 傳首頁 HTML（含 Canvas + 按鈕 + CSS + JS）
// ————————————————————————————
void sendIndexPage(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println(R"rawliteral(
<!DOCTYPE html>
<html lang="zh-TW">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32-CAM Control</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background: #fff;
      margin: 20px;
    }
    h1 {
      color: #000;
    }
    .box_1, .box_2, .box_3 {
      width: 100px;
      height: 100px;
      font-size: 25px;
      border: 1px solid #000;
      cursor: pointer;
    }
    .box_2 {
      margin-bottom: 20px;
    }
    .box_3 {
      font-size: 30px;
      margin-left: 50px;
      margin-bottom: 12px;
    }
    .box_5 {
      background: red;
      border-radius: 50%;
      border: none;
      color: #fff;
      font-size: 60px;
      width: 200px;
      height: 200px;
      box-shadow: 5px 5px 10px #504f4f;
      cursor: pointer;
    }
    .box_1:hover, .box_2:hover, .box_3:hover {
      background: #acacac;
      transform: scale(0.95);
    }
    .box_1:active, .box_2:active, .box_3:active {
      transform: scale(0.85);
    }
    .box_5:hover {
      background: #db0000;
    }
    .box_5:active {
      background: #db0000;
      box-shadow: 0 0 50px #ff4d4d;
      transform: scale(0.9);
    }
    canvas {
      border: 2px solid #333;
      border-radius: 5px;
    }
    .word{
      font-size: 20px;
      color: #333;
      margin-top: 10px;
    }
    .layout {
      display: flex;
      gap: 20px;
    }
    .controls {
      display: grid;
      grid-template-columns: repeat(3, 105px);
      gap: 3px;
      justify-content: center;
      margin-top: 10px;
    }
    .servos {
      display: grid;
      grid-template-columns: repeat(3, 100px);
      grid-template-rows: 100px 100px auto;
      gap: 10px;
      justify-content: center;
      align-items: center;
      margin-top: 10px;
    }
    .servos button:nth-child(1) { grid-column: 2; grid-row: 1; }
    .servos button:nth-child(2) { grid-column: 1; grid-row: 2; }
    .servos button:nth-child(3) { grid-column: 2; grid-row: 2; }
    .servos button:nth-child(4) { grid-column: 3; grid-row: 2; }
    .servos .switch-container {
      grid-column: 2;
      grid-row: 3;
      text-align: center;
      margin-top: 20px;
    }
    .switch {
      position: relative;
      display: inline-block;
      width: 60px;
      height: 34px;
    }
    .switch input {
      opacity: 0;
      width: 0;
      height: 0;
    }
    .slider {
      position: absolute;
      cursor: pointer;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background: #ff0000;
      transition: .4s;
      border-radius: 34px;
    }
    .slider:before {
      position: absolute;
      content: '';
      height: 26px;
      width: 26px;
      left: 4px;
      bottom: 4px;
      background: #fff;
      transition: .4s;
      border-radius: 50%;
    }
    input:checked + .slider {
      background: #00ff37;
    }
    input:checked + .slider:before {
      transform: translateX(26px);
    }
  </style>
  <script>
    function cmd(c) {
      fetch('/?cmd=' + c);
    }
    function toggleMode(checkbox) {
      const mode = checkbox.checked ? 'AUTO_MODE' : 'MANUAL_MODE';
      cmd(mode);
      document.getElementById('mode-status').innerText = '目前模式：' + (checkbox.checked ? '自動' : '手動');
    }
    function loop() {
      fetch('/stream')
        .then(r => r.blob())
        .then(b => {
          let img = new Image();
          img.onload = () => {
            let c = document.getElementById('cv');
            c.getContext('2d').drawImage(img, 0, 0, c.width, c.height);
            URL.revokeObjectURL(img.src);
            setTimeout(loop, 66);
          };
          img.src = URL.createObjectURL(b);
        })
        .catch(_ => setTimeout(loop, 200));
    }
    window.onload = loop;
  </script>
</head>
<body>
  <h1>ESP32‑CAM OSD 控制 (YOLOv7‑tiny)</h1>
  <div class="layout">
    <canvas id="cv" width="640" height="480"></canvas>
    <div>
      <div class="controls">
        <button class="box_1" onmousedown="cmd('CAR_TURNL')" onmouseup="cmd('CAR_STOP')" >左旋</button>
        <button class="box_1" onmousedown="cmd('CAR_GO')" onmouseup="cmd('CAR_STOP')" >前進</button>
        <button class="box_1" onmousedown="cmd('CAR_TURNR')" onmouseup="cmd('CAR_STOP')" >右旋</button>
        <button class="box_2" onmousedown="cmd('CAR_LEFT')" onmouseup="cmd('CAR_STOP')" >左移</button>
        <button class="box_2" onmousedown="cmd('CAR_BACK')" onmouseup="cmd('CAR_STOP')" >後退</button>
        <button class="box_2" onmousedown="cmd('CAR_RIGHT')" onmouseup="cmd('CAR_STOP')" >右移</button>
        <button class="box_5"
                onmousedown="cmd('FIRE_ON')"
                onmouseup="cmd('FIRE_OFF')"
                ontouchstart="cmd('FIRE_ON')"
                ontouchend="cmd('FIRE_OFF')">FIRE</button>
      </div>
    </div>
    <div>
      <div class="servos">
        <button class="box_3" onmousedown="cmd('UP')">上</button>
        <button class="box_3" onmousedown="cmd('LEFT_SERVO')"onmouseup="cmd('SERVO_STOP')">左</button>
        <button class="box_3" onmousedown="cmd('DOWN')">下</button>
        <button class="box_3" onmousedown="cmd('RIGHT_SERVO')"onmouseup="cmd('SERVO_STOP')">右</button>
        <div class="switch-container">
          <label class="switch">
            <input type="checkbox" id="modeToggle" onchange="toggleMode(this)">
            <span class="slider"></span>
          </label>
          <div class="word">手動/自動</div>
          <div id="mode-status" style="font-size: 18px; margin-top: 8px; color: #333;">目前模式：手動</div>
        </div>
      </div>
    </div>
  </div>
</body>
</html>
)rawliteral");
}


// ————————————————————————————
// 3) 處理命令：/?cmd=XXXX
// ————————————————————————————
void handleCommand(WiFiClient &client, const String &req) {
  if (req.indexOf("AUTO_MODE") >= 0) {
    aimingMode = true;
    Serial.println("Mode: AUTO (持續辨識)");
  }
  if (req.indexOf("MANUAL_MODE") >= 0) {
    aimingMode = false;
    Serial.println("Mode: MANUAL (不繪製)");
  }
  if (req.indexOf("FIRE_ON") >= 0) {
  if (fireEnabled) {
    digitalWrite(FIRE_LED, HIGH);
    Serial.println("FIRE ON");
  } else {
    Serial.println("FIRE BLOCKED! 類別1在畫面中，禁止開火");
  }
}

  if (req.indexOf("FIRE_OFF") >= 0) { digitalWrite(FIRE_LED, LOW);   Serial.println("FIRE OFF"); }
  if (req.indexOf("CAR_TURNL") >= 0) { 
    digitalWrite(9, 0); 
    digitalWrite(10, 1);
    digitalWrite(19, 0);
    digitalWrite(20, 0);
    digitalWrite(21, 0);
    digitalWrite(22, 1);
    digitalWrite(23, 0);
    digitalWrite(24, 0);  
     
    }
  if (req.indexOf("CAR_GO") >= 0) { 
    digitalWrite(9, 0); 
    digitalWrite(10, 1);
    digitalWrite(19, 0);
    digitalWrite(20, 1);
    digitalWrite(21, 0);
    digitalWrite(22, 1);
    digitalWrite(23, 1);
    digitalWrite(24, 0);  
     
    }
  if (req.indexOf("CAR_TURNR") >= 0) { 
    digitalWrite(9, 0); 
    digitalWrite(10, 0);
    digitalWrite(19, 0);
    digitalWrite(20, 1);
    digitalWrite(21, 0);
    digitalWrite(22, 0);
    digitalWrite(23, 1);
    digitalWrite(24, 0);  
     
    }
  if (req.indexOf("CAR_LEFT") >= 0) { 
    digitalWrite(9, 1); 
    digitalWrite(10, 0);
    digitalWrite(19, 0);
    digitalWrite(20, 1);
    digitalWrite(21, 0);
    digitalWrite(22, 1);
    digitalWrite(23, 0);
    digitalWrite(24, 1);  
    
    }
  if (req.indexOf("CAR_BACK") >= 0) { 
    digitalWrite(9, 1); 
    digitalWrite(10, 0);
    digitalWrite(19, 1);
    digitalWrite(20, 0);
    digitalWrite(21, 1);
    digitalWrite(22, 0);
    digitalWrite(23, 0);
    digitalWrite(24, 1);  
    
    }
  if (req.indexOf("CAR_RIGHT") >= 0) { 
    digitalWrite(9, 0); 
    digitalWrite(10, 1);
    digitalWrite(19, 1);
    digitalWrite(20, 0);
    digitalWrite(21, 1);
    digitalWrite(22, 0);
    digitalWrite(23, 1);
    digitalWrite(24, 0);
    
    }
  if (req.indexOf("CAR_STOP") >= 0) { 
    digitalWrite(9, 0); 
    digitalWrite(10, 0);
    digitalWrite(19, 0);
    digitalWrite(20, 0);
    digitalWrite(21, 0);
    digitalWrite(22, 0);
    digitalWrite(23, 0);
    digitalWrite(24, 0);
    }
  if(!aimingMode){
  if (req.indexOf("UP") >= 0) { 
    tiltAngle = constrain(tiltAngle + 5, 0, 180);
    servoTilt.write(tiltAngle);
    }
  if (req.indexOf("LEFT_SERVO") >= 0) { 
    servoPan.write(60);
    }
  if (req.indexOf("DOWN") >= 0) { 
    tiltAngle = constrain(tiltAngle - 5, 0, 180);
    servoTilt.write(tiltAngle);
    }
  if (req.indexOf("RIGHT_SERVO") >= 0) { 
    servoPan.write(120);
    }
  if (req.indexOf("SERVO_STOP") >= 0) { 
    servoPan.write(90);
    }  
  }      
  client.print("HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n");
  client.stop();
}

// ————————————————————————————
// 4) 處理單張 JPEG：/stream
// ————————————————————————————
void handleStream(WiFiClient &client) {
  Camera.getImage(CHANNEL, &img_addr, &img_len);
  client.print("HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: ");
  client.print(img_len);
  client.print("\r\n\r\n");
  client.write((uint8_t*)img_addr, img_len);
  client.stop();
}

void flushHeaders(WiFiClient &client) {
  while (client.connected() && client.available()) {
    if (client.readStringUntil('\n') == "\r") break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(FIRE_LED, OUTPUT);  digitalWrite(FIRE_LED, LOW);
  // pinMode 車輪控制腳位... etc.
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  //pinMode(11, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);

  servoPan.attach(11);
  servoTilt.attach(12);
  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }

  Camera.configVideoChannel(CHANNEL,    configJPG);
  Camera.configVideoChannel(CHANNEL_NN, configNN);
  Camera.videoInit();
  Camera.channelBegin(CHANNEL);
  Camera.channelBegin(CHANNEL_NN);

  ObjDet.configVideo(configNN);
  ObjDet.setResultCallback(ODPostProcess);
  ObjDet.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV7TINY, NA_MODEL, NA_MODEL);
  ObjDet.begin();                 // 一直 running
  videoStreamerNN.registerInput(Camera.getStream(CHANNEL_NN));
  videoStreamerNN.setStackSize();
  videoStreamerNN.setTaskPriority();
  videoStreamerNN.registerOutput(ObjDet);
  videoStreamerNN.begin();        // 一直 running

  OSD.configVideo(CHANNEL, configJPG);
  OSD.begin();

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  WiFiClient client = server.available();
  if (!client) return;

  String req = client.readStringUntil('\n');
  Serial.print("Req: "); Serial.println(req);
  flushHeaders(client);

  if (req.indexOf("GET /stream") >= 0)      handleStream(client);
  else if (req.indexOf("GET /?cmd=") >= 0)  handleCommand(client, req);
  else                                       sendIndexPage(client);
}
