#include <Arduino.h>

// Motor A connection pins
const int MOTOR_A_PWM = 2;
const int MOTOR_A_IN1 = 24;
const int MOTOR_A_IN2 = 26;

// Motor B connection pins
const int MOTOR_B_PWM = 3;
const int MOTOR_B_IN1 = 28;
const int MOTOR_B_IN2 = 30;

// Sensor pins starting from sensor 1 to 5
const int SENSOR_PINS[] = {40, 41, 42, 43, 44};
const int NUM_SENSORS = sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]);

// PID control constants
const float PID_KP = 110.0;
const float PID_KI = 0.15;
const float PID_KD = 1200.0;

// PID control variables
int sensorValues[NUM_SENSORS];
int pidError = 0;
int previousPidError = 0;
int pidOutput = 0;
int pidIntegral = 0;
int pidDerivative = 0;

// Initial base speed for motors
const int INITIAL_MOTOR_SPEED = 60;

// Read sensor values and calculate error
void readSensorsAndUpdateError() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = digitalRead(SENSOR_PINS[i]);
  }  
  
  // Sensor pattern to PID error mapping
  const int errorMapping[NUM_SENSORS] = {-2, -1, 0, 1, 2};
  bool anySensorActive = false;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] == 0) {  // Assuming 0 is active
      pidError = errorMapping[i];
      anySensorActive = true;
      break;
    }
  }
  
  // Special case when all sensors are inactive
  if (!anySensorActive) {
    pidError = (previousPidError > 0) ? 3 : -3;
  }
  
  previousPidError = pidError; // Store current error for next cycle's derivative
}

// Calculate PID output
void calculatePID() {
  pidIntegral += pidError;
  pidDerivative = pidError - previousPidError;
  pidOutput = (PID_KP * pidError) + (PID_KI * pidIntegral) + (PID_KD * pidDerivative);
}

// Write to motors
void writeMotors(int speedLeft, int speedRight) {
  // Set motor A direction and speed
  digitalWrite(MOTOR_A_IN1, speedLeft < 0);
  digitalWrite(MOTOR_A_IN2, speedLeft >= 0);
  analogWrite(MOTOR_A_PWM, abs(speedLeft));

  // Set motor B direction and speed
  digitalWrite(MOTOR_B_IN1, speedRight < 0);
  digitalWrite(MOTOR_B_IN2, speedRight >= 0);
  analogWrite(MOTOR_B_PWM, abs(speedRight));
}

// Control motors with PID output
void controlMotorsWithPID() {
  int leftMotorSpeed = INITIAL_MOTOR_SPEED - pidOutput;
  int rightMotorSpeed = INITIAL_MOTOR_SPEED + pidOutput;

  // Constrain the speeds to valid PWM range
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  writeMotors(leftMotorSpeed, rightMotorSpeed);
}

void setup() {
  // Setup motor and sensor pins as outputs/inputs
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }
}

void loop() {
  readSensorsAndUpdateError();  // Get sensor readings and update PID error
  calculatePID();               // Calculate PID output based on current error
  controlMotorsWithPID();       // Adjust motors speed based on PID output
}
