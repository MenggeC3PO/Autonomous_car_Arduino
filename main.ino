#include <Servo.h>

// Define constants for the pin numbers
const int AIN1 = 24;
const int AIN2 = 26;
const int PWMA = 2;
const int BIN1 = 28;
const int BIN2 = 30;
const int PWMB = 3;

// Servo objects
Servo servo_duoji1;
Servo servo_duoji2;
Servo servo_duoji3;

// PID constants
const float kp = 110;
const float ki = 0.15;
const float kd = 1200;

// Variables for PID controller
float error, errorLast, errorInte;

// Echo trace function declaration
int echoTrace();

// Other function declarations
float checkdistance_A1_A2();
float checkdistance_A3_A4();
float checkdistance_A5_A6();
void doDcSpeed(int spdL, int spdR);
float PID(float input);
void stateMachine(int a);
void bizhang();
void go_back1();
void turnR90();
void zhizou();
void zhizoul();
void getwuping();
void putwuping();
void anniu();

void setup() {
  pinMode(A1, OUTPUT);
  pinMode(A2, INPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, INPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  for (int i = 30; i <= 34; i++) {
    pinMode(i, INPUT);
  }
  servo_duoji3.attach(duoji3);
  servo_duoji2.attach(duoji2);
  servo_duoji1.attach(duoji1);
}

void loop() {
  doDcSpeed(150,150); 
  pos = echoTrace();
  if(state==1)
  { 
    stateMachine(pos);  
  }
  else if(state==2)
  {
    //doDcSpeed(150,150);
    bizhang();
    if(pos!=0){
    while(state==2){
    stateMachine(pos); } 
     }
   }
  else if(state==3){
    getwuping();
    state++;
  }
  else if(state==4){
    //doDcSpeed(150,150);
    bizhang();
    if(pos!=0){
    while(state==4){
    stateMachine(pos); } 
     }
    }
    else if(state==5){
      putwuping();
      state++;
      }
    else if(state==6){
       bizhang();
    if(pos!=0){
    turnR90;
    while(state==6){
    stateMachine(pos); } 
     }
    }
    else if(state==7)
    {
      anniu();
      }
}

// Implementations of support functions
int echoTrace() {
  int ret = 0;
  for (int i = 0; i < 5; i++) {
    a[i] = digitalRead(30 + i);
    if (a[i] == 0)
      ret += (0x1 << i);
  }
  return ret;
}
float checkdistance_A1_A2() {
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  float distance = pulseIn(A2, HIGH) / 58.00;
  delay(10);
  return distance;
}

float checkdistance_A3_A4() {
  digitalWrite(A3, LOW);
  delayMicroseconds(2);
  digitalWrite(A3, HIGH);
  delayMicroseconds(10);
  digitalWrite(A3, LOW);
  float distance = pulseIn(A4, HIGH) / 58.00;
  delay(10);
  return distance;
}

float checkdistance_A5_A6() {
  digitalWrite(A5, LOW);
  delayMicroseconds(2);
  digitalWrite(A5, HIGH);
  delayMicroseconds(10);
  digitalWrite(A5, LOW);
  float distance = pulseIn(A6, HIGH) / 58.00;
  delay(10);
  return distance;
}
void doDcSpeed(int spdL, int spdR) {
  if (spdL < 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, -spdL);
  }
  else {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, spdL);
  }
  if (spdR < 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, -spdR);
  }
  else {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, spdR);
  }
}
float PID(float input) {
  // Declare static variables to retain their values between function calls
  static float errorLast = 0.0f;
  static float errorInte = 0.0f;
  
  // Constants for PID coefficients
  // Ensure that these are initialized properly in your actual code
  const float kp = 1.0f; // Placeholder proportional gain
  const float ki = 1.0f; // Placeholder integral gain
  const float kd = 1.0f; // Placeholder derivative gain
  
  // Calculate error as a weighted average to smooth out the noise
  float error = error * 0.7f + input * 0.3f;
  
  // Calculate the difference in error for the derivative term
  float errorDiff = error - errorLast;
  
  // Update the integral term with error accumulation and constrain its value
  // to prevent integral windup
  errorInte = constrain(errorInte + error, -50.0f, 50.0f);
  
  float output = kp * error + ki * errorInte + kd * errorDiff;
  errorLast = error;
  return output;
}
void stateMachine(int a) {
  switch (a) {
    case B00000:
      break;
    case B11110:
    case B01111:
    case B01110:
    case B11111:
    {
        state++;      
      break;
     }
    case B00010:
    case B00110:
      //outlineCnt = 0;
      bias = 1;
      break;
    case B00001:
    case B00011:
      //outlineCnt = 0;
      bias = 2;
      break;
    case B00100:
      //outlineCnt = 0;
      bias = 0;
      break;
    case B01000:
    case B01100:
      //outlineCnt = 0;
      bias = -1;
      break;
    case B10000:
    case B11000:
      //outlineCnt = 0;
      bias = -2;
      break;
    default:
      //Serial.println(a,BIN);
      //outlineCnt++;
      break;
  }
    float ff = 150;
    float ctrl = -PID(bias);
    doDcSpeed(ff+ctrl,ff-ctrl);
}
void bizhang() {
  sensor1 = checkdistance_A1_A2();
  sensor2 = checkdistance_A3_A4();
  sensor3 = checkdistance_A5_A6();
  delay(200);
  if (sensor1 < 18) {
    bias1 = 1;
  }
  else if (sensor3 < 18) {
    bias1 = -1;
  } 
  else if (sensor2 < 20 && sensor2 >= 10) {
    if (sensor1 < sensor3) {
      bias1 = 2;
    }
    else {
      bias1 = -2;
    }
  }
  else if (sensor2 < 10) {
    while (sensor2 < 20) {
      doDcSpeed(-150,-150);
    }
  }
  float ctrl1;
  ctrl1 = -PID(bias1);
  doDcSpeed(150 + ctrl1, 150 - ctrl1);
}
void go_back1() {
  doDcSpeed(-150,-50);
  delay(1000);
}
void turnR90() {
  doDcSpeed(150,-150);
  delay(500);
}
void zhizou() {
  doDcSpeed(150,150);
  delay(500);
}
void zhizoul() {
  doDcSpeed(50,150);
  delay(1000);
}
void getwuping() {
  doDcSpeed(0,0);
  servo_duoji3.write(180);
  delay(15);
  servo_duoji2.write(150);
  delay(15);
  turnR90();
  zhizou();
  servo_duoji2.write(30);
  delay(15);
  go_back1();
  zhizoul();
}
void putwuping() {
  doDcSpeed(0,0);
  turnR90();
  zhizou();
  servo_duoji2.write(150);
  delay(15);
  go_back1();
  zhizoul();
}
void anniu() {
  doDcSpeed(0,0);
  servo_duoji1.write(90);
  delay(15);
  doDcSpeed(150,150);
  delay(50000);
}
