// 最後編輯 2017-3-23
// 初版

// 最後編輯 2017-5-14
// 修正 EEPROM.commit(); 與 EEPROM.h 衝突

// 最後編輯 2017-8-28
// 增加網頁對於 中文 支援指令


#include <Servo.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

// Version
String FW_Version = "TinyPlan Firmware Version 4.20 (2017/8/28)";

// Servos Matrix
const int ALLMATRIX = 9;        // GPIO14 + GPIO12 + GPIO13 + GPIO15 + GPIO16 + GPIO5 + GPIO4 + GPIO2 + Run Time
const int ALLSERVOS = 8;        // GPIO14 + GPIO12 + GPIO13 + GPIO15 + GPIO16 + GPIO5 + GPIO4 + GPIO2

// SG90 Servo PWM Pulse Traveling
const int PWMRES_Min = 1;       // PWM Resolution 1
const int PWMRES_Max = 180;     // PWM Resolution 180
const int SERVOMIN = 500;       // 500
const int SERVOMAX = 2400;      // 2400

// Servo Delay Base Time
const int BASEDELAYTIME = 10;   // 10ms

// AP Password
const char WiFiAPPSK[] = "12345678";

// Motion Data Index
int Servo_PROGRAM;

// Servo ID
int GPIO_ID;
int GPIO14_PWM;
int GPIO12_PWM;
int GPIO13_PWM;
int GPIO15_PWM;
int GPIO16_PWM;
int GPIO5_PWM;
int GPIO4_PWM;
int GPIO2_PWM;

// Backup Servo Value
int Running_Servo_POS [ALLMATRIX];

ESP8266WebServer server(80);

Servo GPIO14SERVO;
Servo GPIO12SERVO;
Servo GPIO13SERVO;
Servo GPIO15SERVO;
Servo GPIO16SERVO;
Servo GPIO5SERVO;
Servo GPIO4SERVO;
Servo GPIO2SERVO;


/*============================================================================*/
// 歸零位置
//////////////////////////////  G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
int Servo_Act_0 [ ] PROGMEM = {  90,  90,  90,  90,  90,  90,  90,  90,  0   };

/*============================================================================*/
// 起始位置
//////////////////////////////  G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
int Servo_Act_1 [ ] PROGMEM = {  90,  90,  90,  90,  90,  90,  90,  90,  0   };


/*============================================================================*/
// 測試動作 1
int Servo_Prg_1_Step = 9;
int Servo_Prg_1 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  30,  90,  90,  90,  90,  90,  90, 120,  800 },
  {  90,  30,  90,  90,  90,  90, 120,  90,  800 },
  {  90,  90,  30,  90,  90, 120,  90,  90,  800 },
  {  90,  90,  90,  30, 120,  90,  90,  90,  800 },

  {  90,  90,  90, 120,  30,  90,  90,  90,  800 },
  {  90,  90, 120,  90,  90,  30,  90,  90,  800 },
  {  90, 120,  90,  90,  90,  90,  30,  90,  800 },
  { 120,  90,  90,  90,  90,  90,  90,  30,  800 },

  {  90,  90,  90,  90,  90,  90,  90,  90,  800 }    //起始位置
};


/*============================================================================*/
// 測試動作 2
int Servo_Prg_2_Step = 5;
int Servo_Prg_2 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  30,  90,  90,  90,  90,  90,  90, 120,  800 },
  {  90,  30,  90,  90,  90,  90, 120,  90,  800 },
  {  90,  90,  30,  90,  90, 120,  90,  90,  800 },
  {  90,  90,  90,  30, 120,  90,  90,  90,  800 },

  {  90,  90,  90,  90,  90,  90,  90,  90,  800 }    //起始位置
};


/*============================================================================*/
// 測試動作 3
int Servo_Prg_3_Step = 5;
int Servo_Prg_3 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  30,  90,  90,  90,  90,  90,  90, 120,  800 },
  {  90,  30,  90,  90,  90,  90, 120,  90,  800 },
  {  90,  90,  30,  90,  90, 120,  90,  90,  800 },
  {  90,  90,  90,  30, 120,  90,  90,  90,  800 },

  {  90,  90,  90,  90,  90,  90,  90,  90,  800 }    //起始位置
};


/*============================================================================*/
// 測試動作 4
int Servo_Prg_4_Step = 5;
int Servo_Prg_4 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  30,  90,  90,  90,  90,  90,  90, 120,  800 },
  {  90,  30,  90,  90,  90,  90, 120,  90,  800 },
  {  90,  90,  30,  90,  90, 120,  90,  90,  800 },
  {  90,  90,  90,  30, 120,  90,  90,  90,  800 },

  {  90,  90,  90,  90,  90,  90,  90,  90,  800 }    //起始位置
};


/*============================================================================*/
// 測試動作 5
int Servo_Prg_5_Step = 5;
int Servo_Prg_5 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  90,  90,  90, 120,  30,  90,  90,  90,  800 },
  {  90,  90, 120,  90,  90,  30,  90,  90,  800 },
  {  90, 120,  90,  90,  90,  90,  30,  90,  800 },
  { 120,  90,  90,  90,  90,  90,  90,  30,  800 },

  {  90,  90,  90,  90,  90,  90,  90,  90,  800 }    //起始位置
};


/*============================================================================*/
// 測試動作 6
int Servo_Prg_6_Step = 5;
int Servo_Prg_6 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  90,  90,  90, 120,  30,  90,  90,  90,  800 },
  {  90,  90, 120,  90,  90,  30,  90,  90,  800 },
  {  90, 120,  90,  90,  90,  90,  30,  90,  800 },
  { 120,  90,  90,  90,  90,  90,  90,  30,  800 },

  {  90,  90,  90,  90,  90,  90,  90,  90,  800 }    //起始位置
};


/*============================================================================*/
// 測試動作 7
int Servo_Prg_7_Step = 5;
int Servo_Prg_7 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  90,  90,  90, 120,  30,  90,  90,  90,  800 },
  {  90,  90, 120,  90,  90,  30,  90,  90,  800 },
  {  90, 120,  90,  90,  90,  90,  30,  90,  800 },
  { 120,  90,  90,  90,  90,  90,  90,  30,  800 },

  {  90,  90,  90,  90,  90,  90,  90,  90,  800 }    //起始位置
};


/*============================================================================*/
// 測試動作 8
int Servo_Prg_8_Step = 5;
int Servo_Prg_8 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  90,  90,  90, 120,  30,  90,  90,  90,  800 },
  {  90,  90, 120,  90,  90,  30,  90,  90,  800 },
  {  90, 120,  90,  90,  90,  90,  30,  90,  800 },
  { 120,  90,  90,  90,  90,  90,  90,  30,  800 },

  {  90,  90,  90,  90,  90,  90,  90,  90,  800 }    //起始位置
};


/*============================================================================*/
void Set_PWM_to_Servo(int iServo, int iValue)
{
  // 讀取 EEPROM 修正誤差
  int NewPWM = iValue + (int8_t)EEPROM.read(iServo);

  NewPWM = map(NewPWM, PWMRES_Min, PWMRES_Max, SERVOMIN, SERVOMAX);

  if (iServo >= 7)
  {
    GPIO2SERVO.write(NewPWM);
  }
  else if (iServo >= 6)
  {
    GPIO4SERVO.write(NewPWM);
  }
  else if (iServo >= 5)
  {
    GPIO5SERVO.write(NewPWM);
  }
  else if (iServo >= 4)
  {
    GPIO16SERVO.write(NewPWM);
  }
  else if (iServo >= 3)
  {
    GPIO15SERVO.write(NewPWM);
  }
  else if (iServo >= 2)
  {
    GPIO13SERVO.write(NewPWM);
  }
  else if (iServo >= 1)
  {
    GPIO12SERVO.write(NewPWM);
  }
  else if (iServo == 0)
  {
    GPIO14SERVO.write(NewPWM);
  }
}


/*============================================================================*/
void Servo_PROGRAM_Zero()
{
  // 清除備份目前馬達數值
  for ( int Index = 0; Index < ALLMATRIX; Index++)
  {
    Running_Servo_POS[Index] = Servo_Act_0[Index];
  }

  // 重新載入馬達預設數值
  for (int iServo = 0; iServo < ALLSERVOS; iServo++)
  {
    Set_PWM_to_Servo(iServo, Running_Servo_POS[iServo]);
    delay(10);
  }
}


/*============================================================================*/
void Servo_PROGRAM_Center()
{
  // 清除備份目前馬達數值
  for ( int Index = 0; Index < ALLMATRIX; Index++)
  {
    Running_Servo_POS[Index] = Servo_Act_1[Index];
  }

  // 重新載入馬達預設數值
  for (int iServo = 0; iServo < ALLSERVOS; iServo++)
  {
    Set_PWM_to_Servo(iServo, Running_Servo_POS[iServo]);
    delay(10);
  }
}


/*============================================================================*/
void Servo_PROGRAM_Run(int iMatrix[][ALLMATRIX],  int iSteps)
{
  int INT_TEMP_A, INT_TEMP_B, INT_TEMP_C;

  for ( int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++)   // iSteps 步驟主迴圈
  {
    // InterTotalTime 此步驟總時間
    int InterTotalTime = iMatrix [ MainLoopIndex ] [ ALLMATRIX - 1 ];

    // InterDelayCounter 此步驟基本延遲次數
    int InterDelayCounter = InterTotalTime / BASEDELAYTIME;

    // 內差次數迴圈
    for ( int InterStepLoop = 0; InterStepLoop < InterDelayCounter; InterStepLoop++)
    {

      for (int ServoIndex = 0; ServoIndex < ALLSERVOS; ServoIndex++)   // 馬達主迴圈
      {
        // 馬達現在位置
        INT_TEMP_A = Running_Servo_POS[ServoIndex];

        // 馬達目標位置
        INT_TEMP_B = iMatrix[MainLoopIndex][ServoIndex];

        // 馬達數值不變
        if (INT_TEMP_A == INT_TEMP_B)
        {
          INT_TEMP_C = INT_TEMP_B;
        }

        // 馬達數值減少
        else if (INT_TEMP_A > INT_TEMP_B)
        {
          // PWM內差值 = map( 執行次數時間累加, 開始時間, 結束時間, 內差起始值, 內差最大值 )
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_A - INT_TEMP_B);

          if (INT_TEMP_A - INT_TEMP_C >= INT_TEMP_B)
          {
            Set_PWM_to_Servo(ServoIndex, INT_TEMP_A - INT_TEMP_C);
          }
        }

        // 馬達數值增加
        else if (INT_TEMP_A < INT_TEMP_B)
        {
          // PWM內差值 = map( 執行次數時間累加, 開始時間, 結束時間, 內差起始值, 內差最大值 )
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_B - INT_TEMP_A);

          if (INT_TEMP_A + INT_TEMP_C <= INT_TEMP_B)
          {
            Set_PWM_to_Servo(ServoIndex, INT_TEMP_A + INT_TEMP_C);
          }
        }

      }

      delay(BASEDELAYTIME);
    }

    // 備份目前馬達數值
    for ( int Index = 0; Index < ALLMATRIX; Index++)
    {
      Running_Servo_POS[Index] = iMatrix[MainLoopIndex][Index];
    }

  }

}


/*============================================================================*/
void writeKeyValue(int8_t key, int8_t value)
{
  EEPROM.write(key, value);
  EEPROM.commit();
}


/*============================================================================*/
int8_t readKeyValue(int8_t key)
{
  Serial.println("read");
  Serial.println(key);

  int8_t value = EEPROM.read(key);
}


/*============================================================================*/
void handleAction(WiFiClient client, String req, HTTPMethod method)
{
  server.send(200, "text/plain", "Hello from TinyPlan !");
}


/*============================================================================*/
void handleSave()
{
  String key = server.arg("key");
  String value = server.arg("value");

  int8_t keyInt = key.toInt();
  int8_t valueInt = value.toInt();

  // Software PWM PIN detach
  GPIO14SERVO.detach();
  GPIO12SERVO.detach();
  GPIO13SERVO.detach();
  GPIO15SERVO.detach();
  GPIO16SERVO.detach();
  GPIO5SERVO.detach();
  GPIO4SERVO.detach();
  GPIO2SERVO.detach();
  delay(50);

  if (keyInt == 100)
  {
    writeKeyValue(0, 0);
    writeKeyValue(1, 0);
    writeKeyValue(2, 0);
    writeKeyValue(3, 0);
    writeKeyValue(4, 0);
    writeKeyValue(5, 0);
    writeKeyValue(6, 0);
    writeKeyValue(7, 0);
  }
  else
  {
    // 確認資料介於 -124 ~ 124
    if (valueInt >= -124 && valueInt <= 124)
    {
      // 儲存校正值
      writeKeyValue(keyInt, valueInt);
    }
  }

  // Software PWM PIN attach
  GPIO14SERVO.attach(14, SERVOMIN, SERVOMAX);
  GPIO12SERVO.attach(12, SERVOMIN, SERVOMAX);
  GPIO13SERVO.attach(13, SERVOMIN, SERVOMAX);
  GPIO15SERVO.attach(15, SERVOMIN, SERVOMAX);
  GPIO16SERVO.attach(16, SERVOMIN, SERVOMAX);
  GPIO5SERVO.attach(5, SERVOMIN, SERVOMAX);
  GPIO4SERVO.attach(4, SERVOMIN, SERVOMAX);
  GPIO2SERVO.attach(2, SERVOMIN, SERVOMAX);
  delay(10);

  server.send(200, "text/html", "(key, value)=(" + key + "," + value + ")");
}


/*============================================================================*/
void handleController()
{
  String pm = server.arg("pm");
  String servo = server.arg("servo");

  if (pm != "")
  {
    Servo_PROGRAM = pm.toInt();
  }

  if (servo != "")
  {
    GPIO_ID = servo.toInt();

    String ival = server.arg("value");

    Set_PWM_to_Servo(GPIO_ID, ival.toInt());
  }

  server.send(200, "text/html", "(pm)=(" + pm + ")");
}


/*============================================================================
  無線傳輸模式
  ============================================================================*/
void handleOnLine()
{
  String m0 = server.arg("m0");
  String m1 = server.arg("m1");
  String m2 = server.arg("m2");
  String m3 = server.arg("m3");
  String m4 = server.arg("m4");
  String m5 = server.arg("m5");
  String m6 = server.arg("m6");
  String m7 = server.arg("m7");
  String t1 = server.arg("t1");

  int Servo_Prg_tmp [][ALLMATRIX] = {

    // GPIO14,     GPIO12,     GPIO13,     GPIO15,     GPIO16,     GPIO5,      GPIO4,      GPIO2,      Run Time
    { m0.toInt(), m1.toInt(), m2.toInt(), m3.toInt(), m4.toInt(), m5.toInt(), m6.toInt(), m7.toInt(), t1.toInt() }

  };

  Servo_PROGRAM_Run(Servo_Prg_tmp, 1);

  server.send(200, "text/html", "(m0, m1)=(" + m0 + "," + m1 + ")");
}


/*============================================================================
  馬達歸零畫面
  ============================================================================*/
void handleZero()
{
  String content = "";
  content += "<html>";
  content += "<head>";
  content += "<title>TinyPlan Zero Check</title>";
  content += " <meta charset=\"UTF-8\">";
  content += " <style type=\"text/css\">";
  content += "  body {";
  content += "    color: white;";
  content += "    background-color: #000000 }";

  content += "  .pm_btn {";
  content += "  width: 160px;";
  content += "  -webkit-border-radius: 5;";
  content += "  -moz-border-radius: 5;";
  content += "  border-radius: 5px;";
  content += "  font-family: Arial;";
  content += "  color: #ffffff;";
  content += "  font-size: 24px;";
  content += "  background: #3498db;";
  content += "  padding: 10px 20px 10px 20px;";
  content += "  text-decoration: none;";
  content += "}";
  content += ".pm_text {";
  content += "width: 160px;";
  content += "-webkit-border-radius: 5;";
  content += "-moz-border-radius: 5;";
  content += "border-radius: 5px;";
  content += "font-family: Arial;";
  content += "font-size: 24px;";

  content += "padding: 10px 20px 10px 20px;";
  content += "text-decoration: none;";
  content += "}";

  content += ".pm_btn:hover {";
  content += "  background: #3cb0fd;";
  content += "  background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "  text-decoration: none;";
  content += "}";

  content += ".pms_btn {";
  content += "width: 240px;";
  content += "-webkit-border-radius: 5;";
  content += "-moz-border-radius: 5;";
  content += "border-radius: 5px;";
  content += "font-family: Arial;";
  content += "color: #ffffff;";
  content += "font-size: 24px;";
  content += "background: #3498db;";
  content += "padding: 10px 20px 10px 20px;";
  content += "text-decoration: none;";
  content += "}";

  content += ".pms_btn:hover {";
  content += "background: #3cb0fd;";
  content += "background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "text-decoration: none;";
  content += "}";
  content += "  </style>";
  content += "</head>";
  content += "<body>";

  content += "<table>";
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(0,90)\">D14</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(4,90)\">D16</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(1,90)\">D12</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(5,90)\">D5</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(2,90)\">D13</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(6,90)\">D4</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(3,90)\">D15</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(7,90)\">D2</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "<table>";
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"controlPm(100)\">ALL</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "</body>";
  content += "<script>";

  content += "function controlServo(id, value) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "     document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"controller?servo=\"+id+\"&value=\"+value, true);";
  content += "  xhttp.send();";
  content += "}";

  content += "function controlPm(value) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "     document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"controller?pm=\"+value, true);";
  content += "  xhttp.send();";
  content += "}";

  content += "</script>";
  content += "</html>";

  server.send(200, "text/html", content);
}


/*============================================================================
  簡易動作編輯畫面
  ============================================================================*/
void handleEditor()
{
  String content = "";
  content += "<html>";
  content += "<head>";
  content += "  <title>TinyPlan Motion Editor</title>";
  content += " <meta charset=\"UTF-8\">";
  content += "  <style type=\"text/css\">";
  content += "  body {";
  content += "    color: white;";
  content += "    background-color: #000000 }";

  content += "  .pm_btn {";
  content += "  width: 160px;";
  content += "  -webkit-border-radius: 5;";
  content += "  -moz-border-radius: 5;";
  content += "  border-radius: 5px;";
  content += "  font-family: Arial;";
  content += "  color: #ffffff;";
  content += "  font-size: 24px;";
  content += "  background: #3498db;";
  content += "  padding: 10px 20px 10px 20px;";
  content += "  text-decoration: none;";
  content += "}";
  content += ".pm_text {";
  content += "width: 160px;";
  content += "-webkit-border-radius: 5;";
  content += "-moz-border-radius: 5;";
  content += "border-radius: 5px;";
  content += "font-family: Arial;";
  content += "font-size: 24px;";

  content += "padding: 10px 20px 10px 20px;";
  content += "text-decoration: none;";
  content += "}";

  content += ".pm_btn:hover {";
  content += "  background: #3cb0fd;";
  content += "  background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "  text-decoration: none;";
  content += "}";

  content += ".pms_btn {";
  content += "width: 240px;";
  content += "-webkit-border-radius: 5;";
  content += "-moz-border-radius: 5;";
  content += "border-radius: 5px;";
  content += "font-family: Arial;";
  content += "color: #ffffff;";
  content += "font-size: 24px;";
  content += "background: #3498db;";
  content += "padding: 10px 20px 10px 20px;";
  content += "text-decoration: none;";
  content += "}";

  content += ".pms_btn:hover {";
  content += "background: #3cb0fd;";
  content += "background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "text-decoration: none;";
  content += "}";
  content += "  </style>";
  content += "</head>";
  content += "<body>";

  content += "<table>";
  content += "<tr>";
  content += "<td>D14  ,  Default value = 90<br/><input class=\"pm_text\" type=\"text\" id=\"servo_0\" value=\"90\"><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(0,'servo_0')\">SEND</button></td>";
  content += "<td>D16  ,  Default value = 90<br/><input class=\"pm_text\" type=\"text\" id=\"servo_4\" value=\"90\"><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(4,'servo_4')\">SEND</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>D12  ,  Default value = 90<br/><input class=\"pm_text\" type=\"text\" id=\"servo_1\" value=\"90\"><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(1,'servo_1')\">SEND</button></td>";
  content += "<td>D5  ,  Default value = 90<br/><input class=\"pm_text\" type=\"text\" id=\"servo_5\" value=\"90\"><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(5,'servo_5')\">SEND</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>D13  ,  Default value = 90<br/><input class=\"pm_text\" type=\"text\" id=\"servo_2\" value=\"90\"><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(2,'servo_2')\">SEND</button></td>";
  content += "<td>D4  ,  Default value = 90<br/><input class=\"pm_text\" type=\"text\" id=\"servo_6\" value=\"90\"><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(6,'servo_6')\">SEND</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>D15  ,  Default value = 90<br/><input class=\"pm_text\" type=\"text\" id=\"servo_3\" value=\"90\"><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(3,'servo_3')\">SEND</button></td>";
  content += "<td>D2  ,  Default value = 90<br/><input class=\"pm_text\" type=\"text\" id=\"servo_7\" value=\"90\"><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(7,'servo_7')\">SEND</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "<table>";
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"controlPm(99)\">STANDBY POSE</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "</body>";
  content += "<script>";

  content += "function controlServo(id, textId) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  var value = document.getElementById(textId).value;";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "     document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\",\"controller?servo=\"+id+\"&value=\"+value, true);";
  content += "  xhttp.send();";
  content += "}";

  content += "function controlPm(value) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "     document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"controller?pm=\"+value, true);";
  content += "  xhttp.send();";
  content += "}";

  content += "</script>";
  content += "</html>";
  server.send(200, "text/html", content);
}


/*============================================================================
  馬達校正畫面
  ============================================================================*/
void handleSetting()
{
  int servo7Val = readKeyValue(7);
  String servo7ValStr = String(servo7Val);

  int servo6Val = readKeyValue(6);
  String servo6ValStr = String(servo6Val);

  int servo5Val = readKeyValue(5);
  String servo5ValStr = String(servo5Val);

  int servo4Val = readKeyValue(4);
  String servo4ValStr = String(servo4Val);

  int servo3Val = readKeyValue(3);
  String servo3ValStr = String(servo3Val);

  int servo2Val = readKeyValue(2);
  String servo2ValStr = String(servo2Val);

  int servo1Val = readKeyValue(1);
  String servo1ValStr = String(servo1Val);

  int servo0Val = readKeyValue(0);
  String servo0ValStr = String(servo0Val);

  String content = "";
  content += "<html>";
  content += "<head>";
  content += "  <title>TinyPlan Setting</title>";
  content += " <meta charset=\"UTF-8\">";
  content += "  <style type=\"text/css\">";
  content += "  body {";
  content += "    color: white;";
  content += "    background-color: #000000 }";

  content += "  .pm_btn {";
  content += "  width: 120px;";
  content += "  -webkit-border-radius: 5;";
  content += "  -moz-border-radius: 5;";
  content += "  border-radius: 5px;";
  content += "  font-family: Arial;";
  content += "  color: #ffffff;";
  content += "  font-size: 24px;";
  content += "  background: #3498db;";
  content += "  padding: 10px 20px 10px 20px;";
  content += "  text-decoration: none;";
  content += "}";
  content += ".pm_text {";
  content += "width: 100px;";
  content += "-webkit-border-radius: 5;";
  content += "-moz-border-radius: 5;";
  content += "border-radius: 5px;";
  content += "font-family: Arial;";
  content += "font-size: 24px;";

  content += "padding: 10px 20px 10px 20px;";
  content += "text-decoration: none;";
  content += "}";

  content += ".pm_btn:hover {";
  content += "  background: #3cb0fd;";
  content += "  background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "  text-decoration: none;";
  content += "}";

  content += ".pms_btn {";
  content += "width: 160px;";
  content += "-webkit-border-radius: 5;";
  content += "-moz-border-radius: 5;";
  content += "border-radius: 5px;";
  content += "font-family: Arial;";
  content += "color: #ffffff;";
  content += "font-size: 24px;";
  content += "background: #3498db;";
  content += "padding: 10px 20px 10px 20px;";
  content += "text-decoration: none;";
  content += "}";

  content += ".pms_btn:hover {";
  content += "background: #3cb0fd;";
  content += "background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "text-decoration: none;";
  content += "}";
  content += "  </style>";
  content += "</head>";
  content += "<body>";

  content += "<table>";
  content += "<tr>";
  content += "<td>D14<br/><input class=\"pm_text\" type=\"text\" id=\"servo_0\" value=\"" + servo0ValStr + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(0,'servo_0')\">SET</button></td>";
  content += "<td>D16<br/><input class=\"pm_text\" type=\"text\" id=\"servo_4\" value=\"" + servo4ValStr + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(4,'servo_4')\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>D12<br/><input class=\"pm_text\" type=\"text\" id=\"servo_1\" value=\"" + servo1ValStr + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(1,'servo_1')\">SET</button></td>";
  content += "<td>D5<br/><input class=\"pm_text\" type=\"text\" id=\"servo_5\" value=\"" + servo5ValStr + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(5,'servo_5')\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>D13<br/><input class=\"pm_text\" type=\"text\" id=\"servo_2\" value=\"" + servo2ValStr + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(2,'servo_2')\">SET</button></td>";
  content += "<td>D4<br/><input class=\"pm_text\" type=\"text\" id=\"servo_6\" value=\"" + servo6ValStr + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(6,'servo_6')\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>D15<br/><input class=\"pm_text\" type=\"text\" id=\"servo_3\" value=\"" + servo3ValStr + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(3,'servo_3')\">SET</button></td>";
  content += "<td>D2<br/><input class=\"pm_text\" type=\"text\" id=\"servo_7\" value=\"" + servo7ValStr + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(7,'servo_7')\">SET</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";
  content += "<br>";

  content += "<table>";
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"saveServo(100, 0)\">RESET</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "</body>";
  content += "<script>";

  content += "function saveServo(id, textId) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  var value = \"0\";";
  content += "  if(id==100){";
  content += "    document.getElementById(\"servo_7\").value = \"0\";";
  content += "    document.getElementById(\"servo_6\").value = \"0\";";
  content += "    document.getElementById(\"servo_5\").value = \"0\";";
  content += "    document.getElementById(\"servo_4\").value = \"0\";";
  content += "    document.getElementById(\"servo_3\").value = \"0\";";
  content += "    document.getElementById(\"servo_2\").value = \"0\";";
  content += "    document.getElementById(\"servo_1\").value = \"0\";";
  content += "    document.getElementById(\"servo_0\").value = \"0\";";
  content += "  }else{";
  content += "    value = document.getElementById(textId).value;";
  content += "   }";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "     document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\",\"save?key=\"+id+\"&value=\"+value, true);";
  content += "  xhttp.send();";
  content += "}";

  content += "</script>";
  content += "</html>";
  server.send(200, "text/html", content);
}


/*============================================================================
  主控畫面
  ============================================================================*/
void handleIndex()
{
  String content = "";
  content += "<html>";

  content += "<head>";
  content += "  <title>TinyPlan Controller</title>";
  content += " <meta charset=\"UTF-8\">";
  content += "  <style type=\"text/css\">";
  content += "  body {";
  content += "    color: white;";
  content += "    background-color: #000000 }";

  content += "  .pm_btn {";
  content += "  width: 160px;";
  content += "  -webkit-border-radius: 5;";
  content += "  -moz-border-radius: 5;";
  content += "  border-radius: 5px;";
  content += "  font-family: Arial;";
  content += "  color: #ffffff;";
  content += " font-size: 24px;";
  content += "background: #3498db;";
  content += "  padding: 10px 20px 10px 20px;";
  content += "  text-decoration: none;";
  content += "}";

  content += ".pm_btn:hover {";
  content += "  background: #3cb0fd;";
  content += "  background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "  text-decoration: none;";
  content += "}";

  content += ".pms_btn {";
  content += "width: 240px;";
  content += "-webkit-border-radius: 5;";
  content += "-moz-border-radius: 5;";
  content += "border-radius: 5px;";
  content += "font-family: Arial;";
  content += "color: #ffffff;";
  content += "font-size: 24px;";
  content += "background: #3498db;";
  content += "padding: 10px 20px 10px 20px;";
  content += "text-decoration: none;";
  content += "}";

  content += ".pms_btn:hover {";
  content += "background: #3cb0fd;";
  content += "background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "text-decoration: none;";
  content += "}";
  content += " </style>";
  content += "</head>";


  content += "<body>";
  content += "<table>";
  content += "<tr>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(1)\">Program 1</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(2)\">Program 2</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(3)\">Program 3</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(4)\">Program 4</button></td>";
  content += "<td><button  class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"controlPm(99)\">STANDBY</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(5)\">Program 6</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(6)\">Program 7</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(7)\">Program 8</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(8)\">Program 9</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "<table>";
  content += "<tr>";
  content += "<td>\"" + FW_Version + "\"</td>";
  content += "</tr>";
  content += "</table>";

  content += "</body>";
  content += "<script>";
  content += "function controlPm(id) {";
  content += "var xhttp = new XMLHttpRequest();";
  content += "xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";

  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"controller?pm=\"+id, true);";
  content += "  xhttp.send();";
  content += "}";
  content += "function controlPms(id) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";

  content += " }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"controller?pms=\"+id, true);";
  content += "  xhttp.send();";
  content += "}";
  content += "</script>";
  content += "</html>";

  server.send(200, "text/html", content);
}


/*============================================================================*/
void enableWebServer()
{
  HTTPMethod getMethod = HTTP_GET;

  server.on("/controller", getMethod, handleController);
  server.on("/save", getMethod, handleSave);

  server.on("/", getMethod, handleIndex);
  server.on("/editor", getMethod, handleEditor);
  server.on("/zero", getMethod, handleZero);
  server.on("/setting", getMethod, handleSetting);

  server.on("/online", getMethod, handleOnLine);

  server.begin();
}


/*============================================================================*/
void setup(void) {

  Serial.begin(9600);
  Serial.println("TinyPlan Start");


  // Software PWM PIN
  GPIO14SERVO.attach(14, SERVOMIN, SERVOMAX);
  GPIO12SERVO.attach(12, SERVOMIN, SERVOMAX);
  GPIO13SERVO.attach(13, SERVOMIN, SERVOMAX);
  GPIO15SERVO.attach(15, SERVOMIN, SERVOMAX);

  GPIO16SERVO.attach(16, SERVOMIN, SERVOMAX);
  GPIO5SERVO.attach(5, SERVOMIN, SERVOMAX);
  GPIO4SERVO.attach(4, SERVOMIN, SERVOMAX);
  GPIO2SERVO.attach(2, SERVOMIN, SERVOMAX);


  // AP SSID Name
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();

  String AP_NameString = "TinyPlan V4 - " + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i = 0; i < AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSK);
  IPAddress myIP = WiFi.softAPIP();


  // EEPROM
  EEPROM.begin(512);
  delay(10);


  // 清除備份目前馬達數值
  for ( int Index = 0; Index < ALLMATRIX; Index++)
  {
    Running_Servo_POS[Index] = Servo_Act_0[Index];
  }


  // 網頁形成
  enableWebServer();
}



/*============================================================================*/
void loop(void) {

  // 網頁建構
  server.handleClient();


  if (Servo_PROGRAM >= 1 )
  {
    switch (Servo_PROGRAM)
    {
      case 1:  // 測試動作 1
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        Servo_PROGRAM_Center();
        break;
      case 2:  // 測試動作 2
        Servo_PROGRAM_Run(Servo_Prg_2, Servo_Prg_2_Step);
        Servo_PROGRAM_Center();
        break;
      case 3:  // 測試動作 3
        Servo_PROGRAM_Run(Servo_Prg_3, Servo_Prg_3_Step);
        Servo_PROGRAM_Center();
        break;
      case 4:  // 測試動作 4
        Servo_PROGRAM_Run(Servo_Prg_4, Servo_Prg_4_Step);
        Servo_PROGRAM_Center();
        break;
      case 5:  // 測試動作 5
        Servo_PROGRAM_Run(Servo_Prg_5, Servo_Prg_5_Step);
        Servo_PROGRAM_Center();
        break;
      case 6:  // 測試動作 6
        Servo_PROGRAM_Run(Servo_Prg_6, Servo_Prg_6_Step);
        Servo_PROGRAM_Center();
        break;
      case 7:  // 測試動作 7
        Servo_PROGRAM_Run(Servo_Prg_7, Servo_Prg_7_Step);
        Servo_PROGRAM_Center();
        break;
      case 8:  // 測試動作 8
        Servo_PROGRAM_Run(Servo_Prg_8, Servo_Prg_8_Step);
        Servo_PROGRAM_Center();
        break;
      case 99: // 待機
        Servo_PROGRAM_Center();
        delay(300);
        break;
      case 100:  // 達文西姿勢
        Servo_PROGRAM_Zero();
        delay(300);
        break;
    }
    Servo_PROGRAM = 0;
  }

}



