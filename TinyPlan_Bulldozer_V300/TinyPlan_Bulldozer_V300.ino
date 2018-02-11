// 最後編輯 2017-4-26
// 自動歸零 增加組裝便利性
// D14 LEFT_MOTOR，D12 RIGHT_MOTOR，D13 BUCKET_MOTOR

// 最後編輯 2017-5-14
// 修正 EEPROM.commit(); 與 EEPROM.h 衝突

// 最後編輯 2017-12-03
// 增用 WebSocketsServer


#include <Servo.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>


// Version
String FW_Version = "TinyPlan Bulldozer v3.00 (2017/12/03)";

// SG90 Servo PWM Pulse Traveling
const int SERVOMIN = 500;       // 500us
const int SERVOMAX = 2400;      // 2400us

// Commands sent through Web Socket
const char stopbutton[] = "stopbutton";
const char forwardbutton[] = "forwardbutton";
const char backwardbutton[] = "backwardbutton";
const char turnleftbutton[] = "turnleftbutton";
const char turnrightbutton[] = "turnrightbutton";
const char leftslider[] = "leftslider";
const char rightslider[] = "rightslider";

// Servo Trim Value
int Running_Servo_Trim [3];

int Speed = 30;
int LeftServoCen = 90;
int RightServoCen = 90;
int BucketServoCen = 90;

Servo LEFT_MOTOR;    // D14
Servo RIGHT_MOTOR;   // D12
Servo BUCKET_MOTOR;  // D13

ESP8266WebServer server = ESP8266WebServer(80);
WebSocketsServer webSocket = WebSocketsServer(81);


/*============================================================================*/
String INDEX_HTML = "<html>"
                    "<head>"
                    "<script>"
                    "var connection = new WebSocket('ws://'+location.hostname+':81/', ['arduino']);"
                    "connection.onopen = function ()       { connection.send('Connect ' + new Date()); };"
                    "connection.onerror = function (error) { console.log('WebSocket Error ', error);};"
                    "connection.onmessage = function (e)   { console.log('Server: ', e.data);};"
                    "function sendRGB() {"
                    " var r = parseInt(document.getElementById('r').value).toString(16);"
                    " var g = parseInt(document.getElementById('g').value).toString(16);"
                    " var b = parseInt(document.getElementById('b').value).toString(16);"
                    " if(r.length < 2) { r = '0' + r; }"
                    " if(g.length < 2) { g = '0' + g; }"
                    " if(b.length < 2) { b = '0' + b; }"
                    " var rgb = '#'+r+g+b;"
                    " console.log('RGB: ' + rgb);"
                    " connection.send(rgb);"
                    "}"
                    "</script>"
                    "</head>"
                    "<body>"
                    "LED Control:<br/><br/>"
                    "R: <input id='r' type='range' min='0' max='255' step='1' value='0' oninput='sendRGB();'/><br/>"
                    "G: <input id='g' type='range' min='0' max='255' step='1' value='0' oninput='sendRGB();'/><br/>"
                    "B: <input id='b' type='range' min='0' max='255' step='1' value='0'oninput='sendRGB();'/><br/>"
                    "</body>"
                    "</html>";


/*============================================================================*/
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
  switch (type)
  {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;

    case WStype_CONNECTED: {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

        // send message to client
        webSocket.sendTXT(num, "Connected");
      }
      break;

    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);

      if (strcmp(stopbutton, (const char *)payload) == 0) {
        LEFT_MOTOR.write(LeftServoCen + Running_Servo_Trim[0]);
        RIGHT_MOTOR.write(RightServoCen + Running_Servo_Trim[1]);
      }
      else if (strcmp(forwardbutton, (const char *)payload) == 0) {
        LEFT_MOTOR.write(LeftServoCen + Speed + Running_Servo_Trim[0]);
        RIGHT_MOTOR.write(RightServoCen - Speed + Running_Servo_Trim[1]);
      }
      else if (strcmp(backwardbutton, (const char *)payload) == 0) {
        LEFT_MOTOR.write(LeftServoCen - Speed + Running_Servo_Trim[0]);
        RIGHT_MOTOR.write(RightServoCen + Speed + Running_Servo_Trim[1]);
      }
      else if (strcmp(turnleftbutton, (const char *)payload) == 0) {
        LEFT_MOTOR.write(LeftServoCen - Speed + Running_Servo_Trim[0]);
        RIGHT_MOTOR.write(RightServoCen - Speed + Running_Servo_Trim[1]);
      }
      else if (strcmp(turnrightbutton, (const char *)payload) == 0) {
        LEFT_MOTOR.write(LeftServoCen + Speed + Running_Servo_Trim[0]);
        RIGHT_MOTOR.write(RightServoCen + Speed + Running_Servo_Trim[1]);
      }
      else {
        Serial.println("Unknown command");
      }

      if (payload[0] == '#') {
        // get servo data
        //int angle = map(abs(255 - ((uint32_t) strtol((const char *) &payload[1], NULL, 16) >> 16) & 0xFF), 0, 255, 1, 180);
        int angle = map(abs(255 - ((uint32_t) strtol((const char *) &payload[1], NULL, 16) >> 16) & 0xFF), 0, 255, 1, 180);
        Serial.println(angle);
        //BUCKET_MOTOR.write(angle + Running_Servo_Trim[2]);
        BUCKET_MOTOR.write(angle);
      }
      break;
  }
}


/*============================================================================*/
void setup()
{
  Serial.begin(115200);
  delay(10);
  Serial.println("TinyPlan Start");


  // Software PWM PIN
  LEFT_MOTOR.attach(14, SERVOMIN, SERVOMAX);
  RIGHT_MOTOR.attach(12, SERVOMIN, SERVOMAX);
  BUCKET_MOTOR.attach(13, SERVOMIN, SERVOMAX);


  // AP SSID Name
  byte mac[6];
  char macHEX[13];
  String prefix = "TinyPlan Bulldozer-";
  String APpassword = "12345678";

  WiFi.macAddress(mac);
  //sprintf(macHEX,"%02X%02X%02X%02X%02X%02X\0",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  sprintf(macHEX, "%02X%02X\0", mac[4], mac[5]);
  String APssid = prefix + macHEX;
  Serial.println("SSID = " + APssid);
  Serial.println("PW = " + APpassword);

  WiFi.softAP(APssid.c_str(), APpassword.c_str());
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address : ");
  Serial.println(myIP);


  // EEPROM
  EEPROM.begin(512);
  delay(10);


  // Read EEPROM Trim
  Running_Servo_Trim[0] = (int8_t)EEPROM.read(0); // Left Servo
  Running_Servo_Trim[1] = (int8_t)EEPROM.read(1); // Right Servo
  Running_Servo_Trim[2] = (int8_t)EEPROM.read(2); // Bucket Servo


  // Initialize Servo
  LEFT_MOTOR.write(LeftServoCen + Running_Servo_Trim[0]);
  RIGHT_MOTOR.write(RightServoCen + Running_Servo_Trim[1]);
  BUCKET_MOTOR.write(BucketServoCen + Running_Servo_Trim[2]);


  // Start webSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);


  // Handle index
  server.on("/", []() {
    server.send(200, "text/html", INDEX_HTML);
  });


  // Start Server
  server.begin();
}


/*============================================================================*/
void loop()
{
  webSocket.loop();
  server.handleClient();
}

