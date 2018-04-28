// Modify by Mason 2017/10/27
// Add MiniKame motion to TinyPlan, refer to https://www.thingiverse.com/thing:1265766

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

#include <Servo.h>

// Servos Matrix
const int ALLMATRIX = 9;        // GPIO14 + GPIO12 + GPIO13 + GPIO15 + GPIO16 + GPIO5 + GPIO4 + GPIO2 + Run Time
const int ALLSERVOS = 8;        // GPIO14 + GPIO12 + GPIO13 + GPIO15 + GPIO16 + GPIO5 + GPIO4 + GPIO2

// SG90 Servo PWM Pulse Traveling
const int PWMRES_Min = 1;       // PWM Resolution 1
const int PWMRES_Max = 180;     // PWM Resolution 180
const int SERVOMIN = 500;       // 500
const int SERVOMAX = 2400;      // 2400

// Backup Servo Value
float Running_Servo_POS [ALLMATRIX];
float _servo_position[ALLMATRIX];

#define FRONT_RIGHT_HIP 0
#define FRONT_LEFT_HIP 1
#define FRONT_RIGHT_LEG 2
#define FRONT_LEFT_LEG 3
#define BACK_RIGHT_HIP 4
#define BACK_LEFT_HIP 5
#define BACK_RIGHT_LEG 6
#define BACK_LEFT_LEG 7

class MiniKame {

  public:
    MiniKame();
    void init();
    void svodetach();
    void svoattach();
    void run(int dir = 1, float steps = 1, float T = 550);
    void walk(int dir = 1, float steps = 1, float T = 550);
    void omniWalk(float steps, float T, bool side, float turn_factor);
    void turnL(float steps, float period);
    void turnR(float steps, float period);
    void moonwalkL(float steps, float period);
    void dance(float steps, float period);
    void upDown(float steps, float period);
    void pushUp(float steps, float period);
    void hello();
    void jump();
    void home();
    void frontBack(float steps, float period);

    void setServo(int id, float target);
    void reverseServo(int id);
    float getServo(int id);
    void moveServos(int time, float target[8]);
  private:
    Servo servo[8];
    int board_pins[8];
    int trim[8]; //deviation servo offset
    bool reverse[8];
    unsigned long _init_time;
    unsigned long _final_time;
    unsigned long _partial_time;
    float _increment[8];
    //    float _servo_position[8];

    void execute(float steps, float period[8], int amplitude[8], int offset[8], int phase[8]);
    inline int angToUsec(float value) {
      return value / 180 * (SERVOMAX - SERVOMIN) + SERVOMIN;
    };
};


// Version
String FW_Version = "TinyPlan Quadruped v2.0 (2017/10/26)";

MiniKame robot;

// AP Password
const char WiFiAPPSK[] = "12345678";

// Motion Data Index
int Servo_PROGRAM;

ESP8266WebServer server(80);

int GPIO_ID;
unsigned long _ref_time1 = 0;
float _delta_time1 = 0;
float _output1;
float angle;

MiniKame::MiniKame(): reverse{0, 0, 0, 0, 0, 0, 0, 0}, trim{0, 0, 0, 0, 0, 0, 0, 0} {
  board_pins[FRONT_RIGHT_HIP] = 12; // front left inner g12
  board_pins[FRONT_LEFT_HIP] = 13; // front right inner g13
  board_pins[BACK_RIGHT_HIP] = 5; // back left inner g5
  board_pins[BACK_LEFT_HIP] = 4; // back right inner g4
  board_pins[FRONT_RIGHT_LEG] = 14; // front left outer g14
  board_pins[FRONT_LEFT_LEG] = 15; // front right outer g15
  board_pins[BACK_RIGHT_LEG] = 16; // back left outer g16
  board_pins[BACK_LEFT_LEG] = 2; // back right outer g2
}

void MiniKame::svodetach() {
  for (int i = 0; i < 8; i++) {
    servo[i].detach();
  }
}

void MiniKame::svoattach() {
  for (int i = 0; i < 8; i++) {
    servo[i].attach(board_pins[i], SERVOMIN, SERVOMAX);
  }
}


void MiniKame::init() {
  // EEPROM
  EEPROM.begin(512);
  delay(10);

  for (int i = 0; i < 8; i++) {
    servo[i].attach(board_pins[i], SERVOMIN, SERVOMAX);
  }
  trim[0] = (int8_t)EEPROM.read(1);
  trim[1] = (int8_t)EEPROM.read(2);
  trim[2] = (int8_t)EEPROM.read(0);
  trim[3] = (int8_t)EEPROM.read(3);
  trim[4] = (int8_t)EEPROM.read(5);
  trim[5] = (int8_t)EEPROM.read(6);
  trim[6] = (int8_t)EEPROM.read(4);
  trim[7] = (int8_t)EEPROM.read(7);

  home();
}

void MiniKame::turnR(float steps, float T = 600) {
  int x_amp = 15;
  int z_amp = 15;
  int ap = 15;
  //int hi = 23;
  int hi = 0;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi};
  int phase[] = {0, 180, 90, 90, 180, 0, 90, 90};

  execute(steps, period, amplitude, offset, phase);
}

void MiniKame::turnL(float steps, float T = 600) {
  int x_amp = 15;
  int z_amp = 15;
  int ap = 15;
  int hi = 0;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi};
  int phase[] = {180, 0, 90, 90, 0, 180, 90, 90};

  execute(steps, period, amplitude, offset, phase);
}

void MiniKame::dance(float steps, float T = 600) {
  int x_amp = 0;
  int z_amp = 25;
  int ap = 20;
  int hi = 0;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi};
  int phase[] = {0, 0, 0, 270, 0, 0, 90, 180};

  execute(steps, period, amplitude, offset, phase);
}

void MiniKame::frontBack(float steps, float T = 600) {
  int x_amp = 20;
  int z_amp = 15;
  int ap = 20;
  int hi = 0;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi};
  int phase[] = {0, 180, 270, 90, 0, 180, 90, 270};

  execute(steps, period, amplitude, offset, phase);
}

void MiniKame::run(int dir, float steps, float T) {
  int x_amp = 15;
  int z_amp = 15;
  int ap = 15;
  int hi = 0;
  int front_x = 0;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {    90 + ap - front_x,
                      90 - ap + front_x,
                      90 - hi,
                      90 + hi,
                      90 - ap - front_x,
                      90 + ap + front_x,
                      90 + hi,
                      90 - hi
                 };
  int phase[] = {0, 0, 90, 90, 180, 180, 90, 90};
  if (dir == 1) {
    phase[0] = phase[1] = 180;
    phase[4] = phase[5] = 0;
  }
  execute(steps, period, amplitude, offset, phase);
}

void MiniKame::moonwalkL(float steps, float T = 5000) {
  int z_amp = 45;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {0, 0, z_amp, z_amp, 0, 0, z_amp, z_amp};
  int offset[] = {90, 90, 90, 90, 90, 90, 90, 90};
  int phase[] = {0, 0, 0, 120, 0, 0, 180, 290};

  execute(steps, period, amplitude, offset, phase);
}

void MiniKame::upDown(float steps, float T = 5000) {
  int x_amp = 0;
  int z_amp = 35;
  int ap = 20;
  //int hi = 25;
  int hi = 0;
  int front_x = 0;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {    90 + ap - front_x,
                      90 - ap + front_x,
                      90 - hi,
                      90 + hi,
                      90 - ap - front_x,
                      90 + ap + front_x,
                      90 + hi,
                      90 - hi
                 };
  int phase[] = {0, 0, 90, 270, 180, 180, 270, 90};

  execute(steps, period, amplitude, offset, phase);
}

void MiniKame::pushUp(float steps, float T = 600) {
  int z_amp = 30;
  int x_amp = 30;
  int hi = 10;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {0, 0, z_amp, z_amp, 0, 0, 0, 0};
  int offset[] = {90, 90, 90 - hi, 90 + hi, 90 - x_amp, 90 + x_amp, 90 + hi, 90 - hi};
  int phase[] = {0, 0, 0, 180, 0, 0, 0, 180};

  execute(steps, period, amplitude, offset, phase);
}

void MiniKame::hello() {
  float sentado[] = {90 + 15, 90 - 15, 90 - 65, 90 + 65, 90 + 20, 90 - 20, 90 + 10, 90 - 10};
  moveServos(150, sentado);
  delay(200);

  int T = 350;
  float period[] = {T, T, T, T, T, T, T, T};
  int amplitude[] = {0, 50, 0, 50, 0, 0, 0, 0};
  int offset[] = {
    90 + 15, 40,
    90 - 10, 90 + 10,
    90 + 20, 90 - 20,
    90 + 65, 90
  };

  int phase[] = {0, 0, 0, 90, 0, 0, 0, 0};

  execute(4, period, amplitude, offset, phase);

  float goingUp[] = {160, 20, 90, 90, 90 - 20, 90 + 20, 90 + 10, 90 - 10};
  moveServos(500, goingUp);
  delay(200);
}

void MiniKame::jump() {
  //float sentado[] = {90 + 15, 90 - 15, 90 - 65, 90 + 65, 90 + 20, 90 - 20, 90 + 10, 90 - 10};
  float sentado[] = {
    90 + 15, 90 - 15, //front hips servos
    90 - 10, 90 + 10, //front leg servos
    90 + 10, 90 - 10, // back hip servos
    90 + 65, 90 - 65  // back leg servos
  };
  int ap = 20;
  int hi = 35;
  float salto[] = {90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap * 3, 90 + ap * 3, 90 + hi, 90 - hi};

  moveServos(150, sentado);
  delay(200);
  //moveServos(0, salto);
  //delay(100);
  home();
}

void MiniKame::home() {
  int position[] = {50, 130, 130, 50, 130, 50, 50, 130};
  //  int position[] = {90, 90, 90, 90, 90, 90, 90, 90};
  for (int i = 0; i < 8; i++) {
    setServo(i, position[i] );
  }
  // update servo angle
  Running_Servo_POS[0] = _servo_position[2];
  Running_Servo_POS[1] = _servo_position[0];
  Running_Servo_POS[2] = _servo_position[1];
  Running_Servo_POS[3] = _servo_position[3];
  Running_Servo_POS[4] = _servo_position[6];
  Running_Servo_POS[5] = _servo_position[4];
  Running_Servo_POS[6] = _servo_position[5];
  Running_Servo_POS[7] = _servo_position[7];
}

void MiniKame::reverseServo(int id) {
  if (reverse[id])
    reverse[id] = 0;
  else
    reverse[id] = 1;
}


void MiniKame::setServo(int id, float target) {
  if (!reverse[id])
    servo[id].writeMicroseconds(angToUsec(target + trim[id]));
  else
    servo[id].writeMicroseconds(angToUsec(180 - (target + trim[id])));
  //  _servo_position[id] = target + trim[id];
  _servo_position[id] = target ;
  // update servo angle
  Running_Servo_POS[0] = _servo_position[2];
  Running_Servo_POS[1] = _servo_position[0];
  Running_Servo_POS[2] = _servo_position[1];
  Running_Servo_POS[3] = _servo_position[3];
  Running_Servo_POS[4] = _servo_position[6];
  Running_Servo_POS[5] = _servo_position[4];
  Running_Servo_POS[6] = _servo_position[5];
  Running_Servo_POS[7] = _servo_position[7];
}

float MiniKame::getServo(int id) {
  return _servo_position[id];
}

void MiniKame::moveServos(int time, float target[8]) {
  if (time > 10) {
    for (int i = 0; i < 8; i++)  _increment[i] = (target[i] - _servo_position[i]) / (time / 10.0);
    _final_time =  millis() + time;

    while (millis() < _final_time) {
      _partial_time = millis() + 10;
      for (int i = 0; i < 8; i++) setServo(i, _servo_position[i] + _increment[i]);
      while (millis() < _partial_time); //pause
    }
  }
  else {
    for (int i = 0; i < 8; i++) setServo(i, target[i]);
  }
  for (int i = 0; i < 8; i++) _servo_position[i] = target[i];
}

void MiniKame::execute(float steps, float period[8], int amplitude[8], int offset[8], int phase[8]) {

  int offset_tinyplan[] = { -40, 40, 40, -40, 40, -40, -40, 40};

  unsigned long global_time = millis();

  for (int i = 0; i < 8; i++)
  {
    _ref_time1 = global_time;
  }

  _final_time = millis() + period[0] * steps;
  while (millis() < _final_time) {
    for (int i = 0; i < 8; i++) {
      _delta_time1 = (float)((millis() - _ref_time1) % (int)period[i]);
      angle = _delta_time1 * 2 * 3.14159 / period[i] + phase[i] * 2 * 3.14159 / 360;
      _output1 =   (float)amplitude[i] * sin(angle) + offset[i] + offset_tinyplan[i];
      setServo(i, _output1);
    }
    yield();
  }
}



/*============================================================================*/
// 歸零位置
//////////////////////////////  G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
int Servo_Act_0 [ ] PROGMEM = {  90,  90,  90,  90,  90,  90,  90,  90,  500   };

/*============================================================================*/
// 起始位置
//////////////////////////////  G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
int Servo_Act_1 [ ] PROGMEM = { 130, 50, 130, 50, 50, 130, 50, 130, 500 };

/*============================================================================*/
// 前進動作 Forward
int Servo_Prg_1_Step = 14;
int Servo_Prg_1 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   110,  90,  92,  34,  37, 110,  70, 143, 100   },
  {    98,  64,  92,  34,  37, 110,  70, 143, 100   },
  {   135,  56,  92,  34,  37, 110,  70, 143, 100   },
  {   143,  70, 110,  37,  34,  92,  56, 135, 100   },
  {   143,  70, 110,  37,  34,  92,  64,  98, 100   },

  {   143,  70, 110,  37,  34,  92,  90, 110, 100   },
  {   143,  70, 110,  37,  34,  92,  88, 146, 100   },
  {   143,  70, 110,  37,  70,  90,  88, 146, 100   },
  {   143,  70, 110,  37,  82, 116,  88, 146, 100   },
  {   143,  70, 110,  37,  45, 124,  88, 146, 100   },

  {   146,  88, 124,  45,  37, 110,  70, 143, 100   },
  {   146,  88, 116,  82,  37, 110,  70, 143, 100   },
  {   146,  88,  90,  70,  37, 110,  70, 143, 100   },
  {   146,  88,  92,  34,  37, 110,  70, 143, 100   }
};


/*============================================================================*/
// 後退動作 Back
int Servo_Prg_2_Step = 14;
int Servo_Prg_2 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   143,  70, 110,  37,  45, 124,  88, 146, 100   },
  {   143,  70, 110,  37,  82, 116,  88, 146, 100   },
  {   143,  70, 110,  37,  70,  90,  88, 146, 100   },
  {   143,  70, 110,  37,  34,  92,  88, 146, 100   },
  {   143,  70, 110,  37,  34,  92,  88, 140, 100   },

  {   143,  70, 110,  37,  34,  92,  64, 125, 100   },
  {   135,  56,  92,  34,  37, 110,  70, 143, 100   },
  {   114,  56,  92,  34,  37, 110,  70, 143, 100   },
  {   110,  90,  92,  34,  37, 110,  70, 143, 100   },
  {   146,  88,  92,  34,  37, 110,  70, 143, 100   },

  {   146,  88,  90,  70,  37, 110,  70, 143, 100   },
  {   146,  88, 116,  82,  37, 110,  70, 143, 100   },
  {   146,  88, 124,  45,  37, 110,  70, 143, 100   },
  {   143,  70, 110,  37,  45, 124,  88, 146, 100   }
};


/*============================================================================*/
// 左轉動作 Left
int Servo_Prg_3_Step = 14;
int Servo_Prg_3 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   146,  88,  90,  70,  37, 110,  70, 143, 150   },
  {   145,  79, 114,  66,  35, 101,  57, 147, 150   },
  {   145,  79, 123,  33,  35, 101,  57, 147, 150   },
  {   145,  79, 123,  33,  35, 101,  57, 147, 150   },
  {   145,  79, 123,  33,  35, 101,  66, 114, 150   },

  {   143,  70, 110,  37,  34,  92,  90, 110, 150   },
  {   143,  70, 110,  37,  34,  92,  88, 146, 150   },
  {   143,  70, 110,  37,  70,  90,  88, 146, 150   },
  {   147,  57, 101,  35,  66, 114,  79, 145, 150   },
  {   147,  57, 101,  35,  33, 123,  79, 145, 150   },

  {   147,  57, 101,  35,  33, 123,  79, 145, 150   },
  {   114,  66, 101,  35,  33, 123,  79, 145, 150   },
  {   110,  90,  92,  34,  37, 110,  70, 143, 150   },
  {   146,  88,  92,  34,  37, 110,  70, 143, 150   }
};


/*============================================================================*/
// 右轉動作 Right
int Servo_Prg_4_Step = 14;
int Servo_Prg_4 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   110,  90,  92,  34,  37, 110,  70, 143, 150   },
  {   141,  57, 101,  35,  33, 123,  79, 145, 150   },
  {   147,  57, 101,  35,  33, 123,  79, 145, 150   },
  {   147,  57, 101,  35,  33, 123,  79, 145, 150   },
  {   147,  57, 101,  35,  66, 114,  79, 145, 150   },

  {   143,  70, 110,  37,  70,  90,  88, 146, 150   },
  {   143,  70, 110,  37,  34,  92,  88, 146, 150   },
  {   143,  70, 110,  37,  34,  92,  90, 110, 150   },
  {   145,  79, 123,  33,  35, 101,  66, 114, 150   },
  {   145,  79, 123,  33,  35, 101,  57, 147, 150   },

  {   145,  79, 123,  33,  35, 101,  57, 147, 150   },
  {   145,  79, 114,  66,  35, 101,  57, 147, 150   },
  {   146,  88,  92,  40,  37, 110,  70, 143, 150   },
  {   146,  88,  92,  34,  37, 110,  70, 143, 150   }
};



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
void Set_PWM_to_Servo(int iServo, int iValue)
{
  // 讀取 EEPROM 修正誤差
  //  int NewPWM = iValue + (int8_t)EEPROM.read(iServo);
  int NewPWM = iValue ;

  //  NewPWM = map(NewPWM, PWMRES_Min, PWMRES_Max, SERVOMIN, SERVOMAX);

  if (iServo >= 7)
  {
    //   GPIO2SERVO.write(NewPWM);
    robot.setServo(7, NewPWM);
  }
  else if (iServo >= 6)
  {
    //    GPIO4SERVO.write(NewPWM);
    robot.setServo(5, NewPWM);
  }
  else if (iServo >= 5)
  {
    //   GPIO5SERVO.write(NewPWM);
    robot.setServo(4, NewPWM);
  }
  else if (iServo >= 4)
  {
    //   GPIO16SERVO.write(NewPWM);
    robot.setServo(6, NewPWM);
  }
  else if (iServo >= 3)
  {
    //   GPIO15SERVO.write(NewPWM);
    robot.setServo(3, NewPWM);
  }
  else if (iServo >= 2)
  {
    //   GPIO13SERVO.write(NewPWM);
    robot.setServo(1, NewPWM);
  }
  else if (iServo >= 1)
  {
    //    GPIO12SERVO.write(NewPWM);
    robot.setServo(0, NewPWM);
  }
  else if (iServo == 0)
  {
    //    GPIO14SERVO.write(NewPWM);
    robot.setServo(2, NewPWM);
  }
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


/*============================================================================*/
void Servo_PROGRAM_Mason(int iMatrix[][ALLMATRIX],  int iSteps)
{
  float inter_increment[ALLSERVOS];
  unsigned long target_time;
  unsigned long step_time;
  for ( int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++)   // iSteps 步驟主迴圈
  {
    int run_time = iMatrix [ MainLoopIndex ] [ ALLMATRIX - 1 ];
    if (run_time > 10) {
      for (int i = 0; i < ALLSERVOS; i++)
      {
        inter_increment[i] = (iMatrix[MainLoopIndex][i] - Running_Servo_POS[i]) / (run_time / 10.0);
      }

      target_time =  millis() + run_time;

      while (millis() < target_time) {
        step_time = millis() + 10;

        // map servo number
        // G14, G12, G13, G15, G16,  G5,  G4,  G2
        robot.setServo(2, Running_Servo_POS[0] + inter_increment[0]);
        robot.setServo(0, Running_Servo_POS[1] + inter_increment[1]);
        robot.setServo(1, Running_Servo_POS[2] + inter_increment[2]);
        robot.setServo(3, Running_Servo_POS[3] + inter_increment[3]);
        robot.setServo(6, Running_Servo_POS[4] + inter_increment[4]);
        robot.setServo(4, Running_Servo_POS[5] + inter_increment[5]);
        robot.setServo(5, Running_Servo_POS[6] + inter_increment[6]);
        robot.setServo(7, Running_Servo_POS[7] + inter_increment[7]);

        // update servo angle
        for ( int Index = 0; Index < ALLMATRIX; Index++)
        {
          Running_Servo_POS[Index] = Running_Servo_POS[Index] + inter_increment[Index];
        }

        while (millis() < step_time); // wait to 10ms
        yield();
      }
    }  else {
      // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
      robot.setServo(2, iMatrix[MainLoopIndex][0] + inter_increment[0]);
      robot.setServo(0, iMatrix[MainLoopIndex][1] + inter_increment[1]);
      robot.setServo(1, iMatrix[MainLoopIndex][2] + inter_increment[2]);
      robot.setServo(3, iMatrix[MainLoopIndex][3] + inter_increment[3]);
      robot.setServo(6, iMatrix[MainLoopIndex][4] + inter_increment[4]);
      robot.setServo(4, iMatrix[MainLoopIndex][5] + inter_increment[5]);
      robot.setServo(5, iMatrix[MainLoopIndex][6] + inter_increment[6]);
      robot.setServo(7, iMatrix[MainLoopIndex][7] + inter_increment[7]);

    }
    // 備份目前馬達數值
    for ( int Index = 0; Index < ALLMATRIX; Index++)
    {
      Running_Servo_POS[Index] = iMatrix[MainLoopIndex][Index];
    }
    // update servo angle
    _servo_position[0] = Running_Servo_POS[1];
    _servo_position[1] = Running_Servo_POS[2];
    _servo_position[2] = Running_Servo_POS[0];
    _servo_position[3] = Running_Servo_POS[3];
    _servo_position[4] = Running_Servo_POS[5];
    _servo_position[5] = Running_Servo_POS[6];
    _servo_position[6] = Running_Servo_POS[4];
    _servo_position[7] = Running_Servo_POS[7];

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
  robot.svodetach();

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

  robot.svoattach();
  delay(10);

  server.send(200, "text/html", "(key, value)=(" + key + "," + value + ")");
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

  Servo_PROGRAM_Mason(Servo_Prg_tmp, 1);

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

void handleIndex()
{
  String content = "";
  content += "<html>";

  content += "<head>";
  content += "  <title>TinyPlan Controller</title>";
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
  content += "<td></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(1)\">FORWARD</button></td>";
  content += "<td></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(3)\">LEFT</button></td>";
  content += "<td><button  class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"controlPm(99)\">STANDBY</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(4)\">RIGHT</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(5)\">Push Up</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(2)\">BACK</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(6)\">Hello</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(7)\">Dance</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(11)\">Up Down</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(8)\">F_Back</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(9)\">M_Left</button></td>";
  content += "<td><button  class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"controlPm(88)\">Demo</button></td>";
  content += "<td><button  class=\"pm_btn\" type=\"button\" onclick=\"controlPm(10)\">M_Right</button></td>";
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

void setup() {
  Serial.begin(115200);

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

  for ( int Index = 0; Index < ALLMATRIX; Index++)
  {
    Running_Servo_POS[Index] = Servo_Act_0[Index];
  }

  robot.init();

  enableWebServer();
}

void loop() {

  server.handleClient();

  if (Servo_PROGRAM >= 1 )
  {
    switch (Servo_PROGRAM)
    {
      case 1:  // forward
        Servo_PROGRAM_Mason(Servo_Prg_1, Servo_Prg_1_Step);
        Servo_PROGRAM_Mason(Servo_Prg_1, Servo_Prg_1_Step);
        Servo_PROGRAM_Mason(Servo_Prg_1, Servo_Prg_1_Step);
        robot.home();
        break;
      case 2:  // backward
        Servo_PROGRAM_Mason(Servo_Prg_2, Servo_Prg_2_Step);
        Servo_PROGRAM_Mason(Servo_Prg_2, Servo_Prg_2_Step);
        Servo_PROGRAM_Mason(Servo_Prg_2, Servo_Prg_2_Step);
        robot.home();
        break;
      case 3:  // left
        robot.turnL(5, 550);
        robot.home();
        break;
      case 4:  // right
        robot.turnR(5, 550);
        robot.home();
        break;
      case 5:  // push up
        robot.pushUp(2, 5000);
        robot.home();
        break;
      case 6:  // hello
        robot.hello();
        robot.home();
        break;
      case 7:  // dance
        robot.dance(2, 1000);
        robot.home();
        break;
      case 8:  // front back
        robot.frontBack(2, 1000);
        robot.home();
        break;
      case 9:  // moonwalkL
        robot.moonwalkL(4, 2000);
        robot.home();
        break;
      case 10:  // moonwalkR
        robot.run(0);
        robot.run(0);
        robot.home();
        break;
      case 11:  // up down
        robot.upDown(4, 250);
        robot.home();
        break;
      case 88:  // demo
        robot.turnR(10, 550);
        //       robot.walk();
        robot.pushUp(2, 5000);
        robot.run();
        robot.run(0);
        robot.turnR(5, 550);
        robot.turnL(5, 550);
        robot.hello();
        robot.dance(2, 1000);
        robot.frontBack(2, 1000);
        robot.moonwalkL(4, 2000);
        robot.upDown(4, 250);
        robot.home();
        break;
      case 99: // Home
        robot.home();
        delay(300);
        break;
      case 100: // �w���˄�
        Servo_PROGRAM_Zero();
        delay(300);
        break;
    }
    Servo_PROGRAM = 0;
  }
}


