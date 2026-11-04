#include <Servo.h>

// ================= PIN CONFIG =================
#define PWM1 22 // front 
#define Dir1 21 // front
#define PWM2 23 // back
#define Dir2 20 // frontn m 

#define Auto_Manual 15

#define Prox1 38
#define Prox2 39

#define prox1_T 34
#define prox1_B 35
#define prox2_T 40
#define prox2_B 41

#define ML_R 5
#define MR_R 4
#define ProbeF_R 2

#define Mode_R 3   // 🔥 เพิ่ม Mode channel
#define  Auto_F_B_pin 7
#define  Auto_L_R_pin 8
// #define Auto_Probe_F 
// #define Auto_Probe_B 

#define  Relay_A_M_pin 16
#define  Relay_Safety_pin 17

#define Batt_pin A1
#define Gas_pin A0

// #define BumpF_pin 36
// #define BumpR_pin 37

#define ML_Hall1 9
#define ML_Hall2 10

#define MR_Hall1 11
#define MR_Hall2 12

// ================= SENSOR LOGIC =================
#define ACTIVE LOW

// ================= RC VARIABLES =================
volatile uint32_t ch_FB_start = 0;
volatile uint32_t ch_FB_value = 1500;
volatile uint32_t ch_FB_lastTime = 0;

volatile uint32_t ch_LR_start = 0;
volatile uint32_t ch_LR_value = 1500;
volatile uint32_t ch_LR_lastTime = 0;

volatile uint32_t ch_PF_start = 0;
volatile uint32_t ch_PF_value = 1500;
volatile uint32_t ch_PF_lastTime = 0;

volatile uint32_t ch_PB_start = 0;
volatile uint32_t ch_PB_value = 1500;
volatile uint32_t ch_PB_lastTime = 0;

// 🔥 Mode Channel
volatile uint32_t ch_Mode_start = 0;
volatile uint32_t ch_Mode_value = 1500;
volatile uint32_t ch_Mode_lastTime = 0;

// ================= GLOBAL =================
int batt_ADC = 0;
int level_ADC = 0;

int Forward_Backward = 100;
int MR_RL = 100;

int ProbeF_command = 0;
int ProbeB_command = 0;

int bumpF_status = 0;
int bumpR_status = 0;

int encoder_LEFT = 0;
int encoder_RIGHT = 0;

String Status_Prob1 = "";
String Status_Prob2 = "";

int Mode = 1; // 0=None, 1=Manual, 2=Auto

int Dir_ML=0;
int Dir_MR=0;

unsigned long pulseCount_ML =0;
unsigned long pulseCount_MR =0;

const unsigned long interval = 1000;
unsigned long previousMillis = 0;

//=============  Parameter wheel and pulse/sec ===================
const float PULSES_PER_REV = 165.0;   
const float WHEEL_CIRCUM = 2.057;

//================== Hall INTERRUPT ===============
void ML_HallISR(){
  unsigned long t = micros();
  static unsigned long lastML = 0;
  if (t - lastML < 1000) return;
  lastML = t;

  pulseCount_ML++;
  if(digitalRead(ML_Hall2))Dir_ML=-1;
  else Dir_ML=1;

}
void MR_HallISR(){
  unsigned long t = micros();
  static unsigned long lastMR = 0;
  if (t - lastMR < 1000) return;
  lastMR = t;

  pulseCount_MR++;
  if(digitalRead(MR_Hall2))Dir_MR=-1;
  else Dir_MR=1;

}

// ================= PID MOTOR CONTROL (merged from dcvg_control.ino) =================
// Physical wheel mapping confirmed on hardware:
//   pins 11/12 ("MR_Hall1/2" below) = LEFT wheel hall sensor
//   pins 9/10  ("ML_Hall1/2" below) = RIGHT wheel hall sensor
// The ML_/MR_ names are kept as-is (existing code), but they are swapped
// relative to physical left/right — see getLeftWheelSpeed/getRightWheelSpeed.

Servo motorLeft;
Servo motorRight;

// ----- PID Gains (tune these) -----
float Kp_r = 0.3;  //0.2
float Ki_r = 0.0;  //0.065
float Kd_r = 0.02;   //0.0

float Kp_l = 0.2;  //0.65 , 0.5
float Ki_l = 0.0;  //0.0  , 0.05
float Kd_l = 0.02;   //0.04

// ----- Feedforward gains -----
float Kff_r  = 0.60;   // 1.0
float KsFF_r = 0.077;   // 0.055

float Kff_l  = 0.60;   // 0.65
float KsFF_l = 0.085;   // 0.075

const float RATIO = 0.004812;  // distance per pulse (m), tuned against dcvg_control.ino's PID gains

const float DEADBAND_HALF = 25.0;
const float CENTER_PWM_R = 1485.0;
const float CENTER_PWM_L = 1600.0;
const float PWM_RANGE = 400.0;
const float MAX_SPEED = 0.85;
const float SPEED_THRESH  = 0.01;

float speed_l = 0;
float speed_r = 0;

const float ALPHA = 0.4;

unsigned long pidPrevious = 0;
unsigned long pidInterval = 100;  // 10 Hz control loop (dt = 0.10 s)

//=============  AUTO PARAMETER ===================
int probe_down_active = 0; // 0 = Inactive, 1 = Active
int probe_up_active = 0; // 0 = Inactive, 1 = Active

// ----- Per-motor feedback filter state -----
struct FeedbackFilter {
  float prevRaw  = 0;
  float medBuf[3] = {0, 0, 0};
  uint8_t medIdx = 0;
  float filt     = 0;
};

FeedbackFilter filtLeft;
FeedbackFilter filtRight;

float applyFeedbackFilter(FeedbackFilter &f, float rawSpeed, float setpoint);

// ----- PID State per motor -----
struct PIDController {
  float setpoint;       // desired speed (e.g. RPM or m/s)
  float feedback;       // measured speed from encoder

  float error;
  float prevError;
  float integral;
  float output;         // PID output → PWM value

  float integralLimit;  // anti-windup clamp
  bool reverse;
  char side;            // 'l' = left, 'r' = right
};

PIDController pidLeft;
PIDController pidRight; // reverse = True

void pidInit(PIDController &pid, float integralLimit, bool reverse, char side);
void pidCompute(PIDController &pid, float dt);

// 3-sample median filter — rejects single-sample hall glitches
float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ----- Feedback filter: zero-holdoff → median → EMA -----
float applyFeedbackFilter(FeedbackFilter &f, float rawSpeed, float setpoint) {
  if (rawSpeed == 0 && f.prevRaw != 0 && fabs(setpoint) > SPEED_THRESH) {
    rawSpeed = f.prevRaw * 0.5f;
  }
  f.prevRaw = rawSpeed;

  f.medBuf[f.medIdx] = rawSpeed;
  f.medIdx = (f.medIdx + 1) % 3;
  float medVal = median3(f.medBuf[0], f.medBuf[1], f.medBuf[2]);

  f.filt = ALPHA * medVal + (1.0f - ALPHA) * f.filt;
  return f.filt;
}

// ----- Velocity feedforward -----
float feedForward(float setpoint, char side) {
  if (fabs(setpoint) < SPEED_THRESH) return 0.0f;     // commanded stop -> no push
  float Kff  = (side == 'l') ? Kff_l  : Kff_r;
  float KsFF = (side == 'l') ? KsFF_l : KsFF_r;
  float ff = Kff * setpoint;
  ff += (setpoint > 0 ? KsFF : -KsFF);                // push in the direction of travel
  return ff;
}

// ----- Initialize a PID controller -----
void pidInit(PIDController &pid, float integralLimit, bool reverse, char side) {
  pid.setpoint    = 0;
  pid.feedback    = 0;
  pid.error       = 0;
  pid.prevError   = 0;
  pid.integral    = 0;
  pid.output      = 0;
  pid.integralLimit = integralLimit;
  pid.reverse     = reverse;
  pid.side        = side;
}

// ----- Compute PID for one motor -----
void pidCompute(PIDController &pid, float dt) {
  float Kp, Ki, Kd;
  if (pid.side == 'l') {
    Kp = Kp_l;
    Ki = Ki_l;
    Kd = Kd_l;
  } else {
    Kp = Kp_r;
    Ki = Ki_r;
    Kd = Kd_r;
  }

  pid.error = pid.setpoint - pid.feedback;

  // Integral with anti-windup (only accumulate when Ki is active, else hold at 0)
  if (Ki != 0.0f) {
    pid.integral += pid.error * dt;
    pid.integral = constrain(pid.integral, -pid.integralLimit, pid.integralLimit);
  } else {
    pid.integral = 0;
  }

  // Derivative
  float derivative = (dt > 0) ? (pid.error - pid.prevError) / dt : 0;

  // Feedforward
  float ff = feedForward(pid.setpoint, pid.side);

  // Output = feedforward + feedback correction (PID trims the residual)
  float raw = ff + (Kp * pid.error) + (Ki * pid.integral) + (Kd * derivative);
  raw = constrain(raw, -MAX_SPEED, MAX_SPEED);

  float CENTER_PWM = (pid.side == 'r') ? CENTER_PWM_R : CENTER_PWM_L;

  // Map to servo PWM
  if(pid.reverse) {
    if (abs(raw) < SPEED_THRESH) {
      pid.output = CENTER_PWM;
    } else if (raw > 0) {
      pid.output = (CENTER_PWM - DEADBAND_HALF) - (raw / MAX_SPEED) * PWM_RANGE;
    } else {
      pid.output = (CENTER_PWM + DEADBAND_HALF) - (raw / MAX_SPEED) * PWM_RANGE;
    }
  }
  else {
    if (abs(raw) < SPEED_THRESH) {
      pid.output = CENTER_PWM;
    } else if (raw > 0) {
      pid.output = (CENTER_PWM + DEADBAND_HALF) + (raw / MAX_SPEED) * PWM_RANGE;
    } else {
      pid.output = (CENTER_PWM - DEADBAND_HALF) + (raw / MAX_SPEED) * PWM_RANGE;
    }
  }

  pid.output = constrain(pid.output, 1000, 2000);
  pid.prevError = pid.error;
}

// ----- Motor Driving -----
void driveMotor(Servo &motor, float pidOutput) {
  motor.writeMicroseconds((int)pidOutput);
}

// pins 11/12 ("MR") are physically the LEFT wheel hall sensor
float getLeftWheelSpeed(float dt) {
  noInterrupts();
  unsigned long count = pulseCount_MR;
  int dir = Dir_MR;
  encoder_LEFT = -1 * count * Dir_MR;
  pulseCount_MR = 0;
  interrupts();
  return (dt > 0) ? (count * RATIO * dir / dt) : 0;
}

// pins 9/10 ("ML") are physically the RIGHT wheel hall sensor
float getRightWheelSpeed(float dt) {
  noInterrupts();
  unsigned long count = pulseCount_ML;
  int dir = Dir_ML;
  encoder_RIGHT = count * Dir_ML;
  pulseCount_ML = 0;
  interrupts();
  return (dt > 0) ? (count * RATIO * dir / dt) : 0;
}

// ================= RC INTERRUPT =================
void ISR_FB() {
  if (digitalRead(ML_R)) ch_FB_start = micros();
  else {
    ch_FB_value = micros() - ch_FB_start;
    ch_FB_lastTime = millis();
  }
}

void ISR_LR() {
  if (digitalRead(MR_R)) ch_LR_start = micros();
  else {
    ch_LR_value = micros() - ch_LR_start;
    ch_LR_lastTime = millis();
  }
}

void ISR_PF() {
  if (digitalRead(ProbeF_R)) ch_PF_start = micros();
  else {
    ch_PF_value = micros() - ch_PF_start;
    ch_PF_lastTime = millis();
  }
}

void ISR_Mode() {
  if (digitalRead(Mode_R)) ch_Mode_start = micros();
  else {
    ch_Mode_value = micros() - ch_Mode_start;
    ch_Mode_lastTime = millis();
  }
}

// ================= FAIL SAFE =================
bool isRCValid() {
  uint32_t now = millis();

  if ((now - ch_FB_lastTime) > 100) return false;
  if ((now - ch_LR_lastTime) > 100) return false;
  if ((now - ch_PF_lastTime) > 100) return false;
  
  if ((now - ch_Mode_lastTime) > 100) return false;

  return true;
}

// ================= PROBE MANUAL =================
void Prob_Action_Manual() {

  if (!isRCValid()) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    //Serial.println("X2");
    return;
  }

  if(digitalRead(prox1_T) != ACTIVE || digitalRead(prox2_T) != ACTIVE) {
    digitalWrite(Relay_Safety_pin,HIGH);
  }
  else {
    digitalWrite(Relay_Safety_pin,LOW);
  }
  Forward_Backward = map(ch_FB_value, 1000, 2000, 200, 0);
  MR_RL = map(ch_LR_value, 1000, 2000, 200, 0);

  // ===== Front =====
  if (ch_PF_value < 1400 && ch_PF_value > 900 &&
      digitalRead(prox1_T) != ACTIVE) {

    digitalWrite(Dir1, HIGH);
    analogWrite(PWM1, 255);
    ProbeF_command = 1;
  }
  else if (ch_PF_value > 1600 &&
           digitalRead(prox1_B) != ACTIVE &&
          digitalRead(Prox1) != ACTIVE) {

    digitalWrite(Dir1, LOW);
    analogWrite(PWM1, 255);
    ProbeF_command = 2;
  }
  else {
    analogWrite(PWM1, 0);
    ProbeF_command = 0;
  }

  // ===== Back =====
  if (ch_PF_value < 1400 && ch_PF_value > 900 &&
      digitalRead(prox2_T) != ACTIVE) {

    digitalWrite(Dir2, HIGH);
    analogWrite(PWM2, 255);
    ProbeB_command = 1;
  }
  else if (ch_PF_value > 1600 &&
           digitalRead(prox2_B) != ACTIVE &&
           digitalRead(Prox2) != ACTIVE) {

    digitalWrite(Dir2, LOW);
    analogWrite(PWM2, 255);
    ProbeB_command = 2;
  }
  else {
    analogWrite(PWM2, 0);
    ProbeB_command = 0;
  }
}

void Prob_Auto_Up() {
  // ===== Front =====
  if (digitalRead(prox1_T) != ACTIVE) {
    digitalWrite(Dir1, HIGH);
    analogWrite(PWM1, 255);
    ProbeF_command = 1;
  }
  else {
    analogWrite(PWM1, 0);
    ProbeF_command = 0;
  }

  // ===== Back =====
  if (digitalRead(prox2_T) != ACTIVE) {
    digitalWrite(Dir2, HIGH);
    analogWrite(PWM2, 255);
    ProbeB_command = 1;
  }
  else {
    analogWrite(PWM2, 0);
    ProbeB_command = 0;
  }
}

void Prob_Auto_Down() {
  // ===== Front =====
  if (digitalRead(prox1_B) != ACTIVE && digitalRead(Prox1) != ACTIVE) {
    digitalWrite(Dir1, LOW);
    analogWrite(PWM1, 255);
    ProbeF_command = 2;
  }
  else {
    analogWrite(PWM1, 0);
    ProbeF_command = 0;
  }
  // ===== Back =====
  if (digitalRead(prox2_B) != ACTIVE && digitalRead(Prox2) != ACTIVE) {
    digitalWrite(Dir2, LOW);
    analogWrite(PWM2, 255);
    ProbeB_command = 2;
  }
  else {
    analogWrite(PWM2, 0);
    ProbeB_command = 0;
  }
}

void Prob_Auto_Main() {
  if (!isRCValid()) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    return;
  }

  if(digitalRead(prox1_T) != ACTIVE || digitalRead(prox2_T) != ACTIVE) {
    digitalWrite(Relay_Safety_pin,HIGH);
  }
  else {
    digitalWrite(Relay_Safety_pin,LOW);
  }
  Forward_Backward = map(ch_FB_value, 1000, 2000, 200, 0);
  MR_RL = map(ch_LR_value, 1000, 2000, 200, 0);

  if(probe_up_active == 0 && probe_down_active == 1) {
    Prob_Auto_Down();
  }
  else if(probe_up_active == 1 && probe_down_active == 0) {
    Prob_Auto_Up();
  }
  else {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    ProbeF_command = 0;
    ProbeB_command = 0;
  }
}

// ================= SEND =================
void Send_Data() {
  Serial.print("$");
  Serial.print(batt_ADC); Serial.print(",");
  Serial.print(level_ADC); Serial.print(",");
  Serial.print(encoder_LEFT); Serial.print(",");
  Serial.print(encoder_RIGHT); Serial.print(",");
  Serial.print(bumpF_status); Serial.print(",");
  Serial.print(bumpR_status); Serial.print(",");
  Serial.print(Status_Prob1); Serial.print(",");
  Serial.print(Status_Prob2); Serial.print(",");
  Serial.print(Forward_Backward); Serial.print(",");
  Serial.print(MR_RL); Serial.print(",");
  Serial.print(ProbeF_command); Serial.print(",");
  Serial.print(ProbeB_command); Serial.print(",");
  Serial.print(Mode);
  Serial.println("#");
}

// ================= SETUP =================
void setup() {

  Serial.begin(115200);

  analogReadResolution(12);
  analogWriteResolution(8);

  pinMode(Auto_F_B_pin,OUTPUT);
  pinMode(Auto_L_R_pin,OUTPUT);
  pinMode(Relay_A_M_pin,OUTPUT);
  pinMode(Relay_Safety_pin,OUTPUT);

  pinMode(PWM1, OUTPUT);
  pinMode(Dir1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(Dir2, OUTPUT);
  pinMode(Auto_Manual,OUTPUT);  

  pinMode(Prox1, INPUT_PULLUP);
  pinMode(Prox2, INPUT_PULLUP);
  pinMode(prox1_T, INPUT_PULLUP);
  pinMode(prox1_B, INPUT_PULLUP);
  pinMode(prox2_T, INPUT_PULLUP);
  pinMode(prox2_B, INPUT_PULLUP);

  // pinMode(BumpF_pin, INPUT);
  // pinMode(BumpR_pin, INPUT);
  pinMode(Batt_pin, INPUT);
  pinMode(Gas_pin, INPUT);
 

  pinMode(ML_R, INPUT);
  pinMode(MR_R, INPUT);
  pinMode(ProbeF_R, INPUT);
  // pinMode(ProbeB_R, INPUT);
  pinMode(Mode_R, INPUT);

  pinMode(ML_Hall1, INPUT);
  pinMode(ML_Hall2, INPUT);
  pinMode(MR_Hall1, INPUT);
  pinMode(MR_Hall2, INPUT);



  attachInterrupt(digitalPinToInterrupt(ML_R), ISR_FB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MR_R), ISR_LR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ProbeF_R), ISR_PF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Mode_R), ISR_Mode, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ML_Hall1), ML_HallISR, RISING);
  attachInterrupt(digitalPinToInterrupt(MR_Hall1), MR_HallISR, RISING);

  // ----- Wheel drive ESCs (auto mode) -----
  // Auto_L_R_pin (8) / Auto_F_B_pin (7) match dcvg_control.ino's LEFT_PWM_PIN/RIGHT_PWM_PIN
  motorLeft.attach(Auto_L_R_pin);
  motorRight.attach(Auto_F_B_pin);
  driveMotor(motorLeft,  1600);
  driveMotor(motorRight, 1500);

  pidInit(pidLeft,  0.75, false, 'l');
  pidInit(pidRight, 0.75, true,  'r');

  pidPrevious = millis();
  Serial.setTimeout(50);
}

// ================= LOOP =================
void loop() {

  // ===== Battery =====
  batt_ADC = analogRead(Batt_pin);
  // batteryVoltage = (batt_ADC * 24.19)/2024.0;
  
  level_ADC = analogRead(Gas_pin);

  unsigned long currentMillis = millis();

  // ===== Auto-mode command intake =====
  // Frame format: "$field0,field1,field2,...#" (comma-separated, count can grow)
  if (Serial.available() > 0) {
    String frame = Serial.readStringUntil('#');
    int startIdx = frame.indexOf('$');
    if (startIdx != -1) {
      const int MAX_FIELDS = 8;   // bump this if you add more fields later
      String fields[MAX_FIELDS];
      int fieldCount = 0;
      int idx = startIdx + 1;
      while (fieldCount < MAX_FIELDS) {
        int nextComma = frame.indexOf(',', idx);
        if (nextComma == -1) {
          fields[fieldCount++] = frame.substring(idx);
          break;
        }
        fields[fieldCount++] = frame.substring(idx, nextComma);
        idx = nextComma + 1;
      }

      if (fieldCount >= 4) {
        speed_l           = fields[0].toFloat();
        speed_r           = fields[1].toFloat();
        probe_up_active   = fields[2].toInt();
        probe_down_active = fields[3].toInt();
        // future field: fields[4].toFloat(), etc.
      }
    }
    while (Serial.available() > 0) {
      Serial.read();  // flush leftover bytes
    }
  }

  // ===== Bump =====
  // bumpF_status = digitalRead(BumpF_pin);
  // bumpR_status = digitalRead(BumpR_pin);

  // ===== Mode Decode =====
  if (ch_Mode_value < 1300) {
    Mode = 1;
    digitalWrite(Relay_A_M_pin,LOW);
    }
  else if (ch_Mode_value > 1700) {
    Mode = 2;
    digitalWrite(Relay_A_M_pin,HIGH);
    }
  else Mode = 0;

  // ===== MODE SELECT =====
  if (Mode == 1) {
    Prob_Action_Manual();
    digitalWrite(Auto_Manual,LOW);

    if (currentMillis - pidPrevious >= pidInterval) {
      float dt = (currentMillis - pidPrevious) / 1000.0f;
      pidPrevious = currentMillis;

      getLeftWheelSpeed(dt);
      getRightWheelSpeed(dt);
    }
  }
  else if (Mode == 2) {
    digitalWrite(Auto_Manual,HIGH);
    Prob_Auto_Main();

    if (currentMillis - pidPrevious >= pidInterval) {
      float dt = (currentMillis - pidPrevious) / 1000.0f;
      pidPrevious = currentMillis;

      pidLeft.setpoint  = -1 * speed_l;
      pidRight.setpoint = speed_r;

      pidLeft.feedback  = applyFeedbackFilter(filtLeft,  getLeftWheelSpeed(dt),  pidLeft.setpoint);
      pidRight.feedback = applyFeedbackFilter(filtRight, getRightWheelSpeed(dt), pidRight.setpoint);

      pidCompute(pidLeft,  dt);
      pidCompute(pidRight, dt);

      driveMotor(motorLeft,  pidLeft.output);
      driveMotor(motorRight, pidRight.output);
    }
  }
  else {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
  }

  // ===== Status =====
  if (digitalRead(prox1_T) == ACTIVE) Status_Prob1 = "1";
  else if (digitalRead(prox1_B) == ACTIVE) Status_Prob1 = "3";
  else if (digitalRead(Prox1) == ACTIVE) Status_Prob1 = "2";
  else Status_Prob1 = "0";

/*****************************************************/
  
  
/********************************************************/

  if (digitalRead(prox2_T) == ACTIVE) Status_Prob2 = "1";
  else if (digitalRead(prox2_B) == ACTIVE) Status_Prob2 = "3";
  else if (digitalRead(Prox2) == ACTIVE) Status_Prob2 = "2";
  else Status_Prob2 = "0";
  

  
  //  digitalWrite(Relay_A_M_pin,LOW);
  // digitalWrite(Relay_Safety_pin,HIGH);
  // delay(1000);
  // digitalWrite(Relay_A_M_pin,HIGH);
  // digitalWrite(Relay_Safety_pin,LOW);
  //  delay(1000);

  Send_Data();
//=================================================================
  // Serial.print("Foword/Backward :");
  // Serial.println(ch_FB_value);
  // Serial.print("Left/Right :");
  // Serial.println(ch_LR_value);
  // Serial.print("Prob :");
  // Serial.println(ch_PF_value);
  // Serial.print("MODE :");
  // Serial.println(ch_Mode_value);

  // Serial.println(digitalRead(Prox1));
  // Serial.println(digitalRead(prox1_T));
  // Serial.println(digitalRead(prox1_B));
  // Serial.println("*************************************");
  // Serial.println(digitalRead(Prox2));
  // Serial.println(digitalRead(prox2_T));
  // Serial.println(digitalRead(prox2_B));
  // Serial.print("Battery :");
  // Serial.println(batt_ADC);

  delay(100);
}