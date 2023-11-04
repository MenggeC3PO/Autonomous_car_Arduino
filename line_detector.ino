#include <EEPROM.h>
#include <Arduino.h>

constexpr char FIRMWARE[] = "LineFollow Robot V1.05\r\n";
constexpr int MOTOR_PINS[] = {24, 26, 2, 28, 30, 3}; // AIN1, AIN2, PWMA, BIN1, BIN2, PWMB
constexpr int SENSOR_PINS[] = {40, 41, 42, 43, 44}; // Sensor digital pins
constexpr int MAX_BUFFER_SIZE = 64;
constexpr int NUM_SENSORS = 5;
constexpr float FILTER_RATIO = 0.3f;

// Motor pins
enum { AIN1, AIN2, PWMA, BIN1, BIN2, PWMB };

int sensorValues[NUM_SENSORS]; // Analog values for each sensor
float lr_ratio = 1.2;
int bias = 0;
int outlineCnt = 0;
float Kp = 110.0f, Ki = 0.15f, Kd = 1200.0f;
float error = 0.0f, errorLast = 0.0f, errorIntegral = 0.0f;

void setupMotors() {
  for (int i = 0; i < 6; ++i) {
    pinMode(MOTOR_PINS[i], OUTPUT);
  }
}

void setupSensors() {
  for (int i = 0; i < NUM_SENSORS; ++i) {
    pinMode(SENSOR_PINS[i], INPUT);
  }
}

void controlMotor(int dirPin1, int dirPin2, int pwmPin, int speed) {
  digitalWrite(dirPin1, speed < 0);
  digitalWrite(dirPin2, speed >= 0);
  analogWrite(pwmPin, abs(speed));
}

void doDcSpeed(int spdL, int spdR) {
  controlMotor(MOTOR_PINS[AIN1], MOTOR_PINS[AIN2], MOTOR_PINS[PWMA], spdL);
  controlMotor(MOTOR_PINS[BIN1], MOTOR_PINS[BIN2], MOTOR_PINS[PWMB], spdR);
}

void stateMachine(int sensorPattern) {
  static const int OUTLINE_THRESHOLD = 10;
  static const int BASE_SPEED = 150;
  
  // Your logic to update bias based on sensorPattern
  
  if (outlineCnt > OUTLINE_THRESHOLD) {
    doDcSpeed(0, 0);
  } else {
    float ctrl = -calculatePid(bias);
    doDcSpeed(BASE_SPEED + ctrl, BASE_SPEED - ctrl);
  }
}

float calculatePid(float input) {
  float errorDiff;
  float output;
  
  error = error * (1.0f - FILTER_RATIO) + input * FILTER_RATIO;
  errorDiff = error - errorLast;
  errorIntegral = constrain(errorIntegral + error, -50.0f, 50.0f);
  output = Kp * error + Ki * errorIntegral + Kd * errorDiff;
  
  errorLast = error;
  return output;
}

int readSensors() {
  int sensorPattern = 0;
  for (int i = 0; i < NUM_SENSORS; ++i) {
    sensorValues[i] = digitalRead(SENSOR_PINS[i]);
    if (sensorValues[i] == LOW) sensorPattern |= (1 << i);
  }
  return sensorPattern;
}

// Echo and other functions
// still in develop

void setup() {
  Serial.begin(115200);
  setupMotors();
  setupSensors();
  echoVersion();
}

char commandBuffer[MAX_BUFFER_SIZE];
int bufferIndex = 0;

void loop() {
  delay(5);
  int sensorPattern = readSensors();
  stateMachine(sensorPattern);
  while (Serial.available()) {
    char incomingByte = Serial.read();
    if (incomingByte == '\n' || bufferIndex >= MAX_BUFFER_SIZE - 1) {
      commandBuffer[bufferIndex] = '\0'; // Null-terminate the string
      // parseCommand(commandBuffer); // Implement parseCommand functionality
      memset(commandBuffer, 0, MAX_BUFFER_SIZE);
      bufferIndex = 0;
    } else {
      commandBuffer[bufferIndex++] = incomingByte;
    }
  }
}
