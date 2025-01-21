#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// MPU6050 Setup
Adafruit_MPU6050 mpu;
sensors_event_t accel, gyro, temp;

// PID Parameters
float kp = 1, ki = 0, kd = 0; // Tune these values
float rollSetpoint = 0, pitchSetpoint = 0, yawSetpoint = 0; // Desired angles
float rollError, pitchError, yawError;
float rollPrevError = 0, pitchPrevError = 0, yawPrevError = 0;
float rollIntegral = 0, pitchIntegral = 0, yawIntegral = 0;

// Motor Control
Servo esc1, esc2, esc3, esc4;
const int escPins[] = {9, 10, 11, 6};
const int minThrottle = 1000;
const int maxThrottle = 2000;
const int baseThrottle = 1800; // Fixed base throttle

// Communication
String command = "";

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  // Attach ESCs
  esc1.attach(escPins[0]);
  esc2.attach(escPins[1]);
  esc3.attach(escPins[2]);
  esc4.attach(escPins[3]);

  // Calibrate ESCs
  calibrateESCs();

  Serial.println("Setup complete!");
}

void calibrateESCs() {
  calibrateESC(esc1);
  calibrateESC(esc2);
  calibrateESC(esc3);
  calibrateESC(esc4);
}

void calibrateESC(Servo &esc) {
  esc.writeMicroseconds(maxThrottle);  // Set maximum throttle for calibration
  delay(2000);  // Wait for ESC to register
  esc.writeMicroseconds(minThrottle);  // Set minimum throttle
  delay(2000);  // Calibration complete
}

void stopMotors() {
  esc1.writeMicroseconds(minThrottle);
  esc2.writeMicroseconds(minThrottle);
  esc3.writeMicroseconds(minThrottle);
  esc4.writeMicroseconds(minThrottle);
  Serial.println("Motors stopped.");
}

void loop() {
  // Check for commands from Raspberry Pi
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("KILL")) {
      stopMotors();
      while (1) {
        delay(100); // Halt further execution
      }
    } else {
      // Try to parse the throttle value
      int newThrottle = command.toInt();
      if (newThrottle > 0) { // Valid throttle value received
        int baseThrottle = newThrottle;  // Set new base throttle value
        Serial.print("New Base Throttle Value: ");
        Serial.println(baseThrottle);
      }
    }
  }
  // Read sensor data
  mpu.getEvent(&accel, &gyro, &temp);

  // Calculate errors
  rollError = rollSetpoint - gyro.gyro.x;
  pitchError = pitchSetpoint - gyro.gyro.y;
  yawError = yawSetpoint - gyro.gyro.z;

  // PID calculations
  float rollOutput = calculatePID(rollError, rollPrevError, rollIntegral, kp, ki, kd);
  float pitchOutput = calculatePID(pitchError, pitchPrevError, pitchIntegral, kp, ki, kd);
  float yawOutput = calculatePID(yawError, yawPrevError, yawIntegral, kp, ki, kd);

  // Motor throttle adjustments
  int motor1Throttle = constrain(baseThrottle + rollOutput * 100 - pitchOutput * 100 + yawOutput * 100, minThrottle, maxThrottle);
  int motor2Throttle = constrain(baseThrottle - rollOutput * 100 - pitchOutput * 100 - yawOutput * 100, minThrottle, maxThrottle);
  int motor3Throttle = constrain(baseThrottle - rollOutput * 100 + pitchOutput * 100 + yawOutput * 100, minThrottle, maxThrottle);
  int motor4Throttle = constrain(baseThrottle + rollOutput * 100 + pitchOutput * 100 - yawOutput * 100, minThrottle, maxThrottle);

  // Set motor throttles
  esc1.writeMicroseconds(motor1Throttle);
  esc2.writeMicroseconds(motor2Throttle);
  esc3.writeMicroseconds(motor3Throttle);
  esc4.writeMicroseconds(motor4Throttle);

  // Pretty print throttle values
  Serial.println("----------------- - Motor Throttle Values ------------------");
  Serial.print("Motor 1 Throttle: "); Serial.println(motor1Throttle);
  Serial.print("Motor 2 Throttle: "); Serial.println(motor2Throttle);
  Serial.print("Motor 3 Throttle: "); Serial.println(motor3Throttle);
  Serial.print("Motor 4 Throttle: "); Serial.println(motor4Throttle);
  Serial.println("----------------------------------------------------------\n");

  // Update previous errors
  rollPrevError = rollError;
  pitchPrevError = pitchError;
  yawPrevError = yawError;

  delay(10); // Loop timing
}

float calculatePID(float error, float &prevError, float &integral, float kp, float ki, float kd) {
  integral += error;
  float derivative = error - prevError;
  float output = kp * error + ki * integral + kd * derivative;
  prevError = error;
  return output;
}



# second code 








/*
 * Autonomous Drone Controller Code
 * Modified to use Serial communication instead of Bluetooth.
 */

#include <Wire.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ---------------- Constants ---------------------------------------
#define INTERRUPT_PIN 2

#define YAW         0
#define PITCH       1
#define ROLL        2
#define THROTTLE    3

#define X           0
#define Y           1
#define Z           2
#define MPU_ADDRESS 0x68

#define STOPPED     0
#define STARTING    1
#define STARTED     2

float instruction[4] = {0, 4, 0, 0};

// ---------------- MPU Variables ---------------------------------------
long gyro_offsets[3] = {130, 44, 35};
long acc_offsets[3] = {-3000, -581, 649};
MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

Quaternion q;
VectorFloat gravity;
float measures[3] = {0, 0, 0};
bool started = false;

int motor_lf = 11, motor_lb = 10, motor_rf = 9, motor_rb = 6;
int motor_lf_throttle = 0, motor_lb_throttle = 0, motor_rf_throttle = 0, motor_rb_throttle = 0;

float errors[3];
float error_sum[3] = {0, 0, 0};
float previous_error[3] = {0, 0, 0};
float Kp[3] = {0, 0.9, 0.9};
float Ki[3] = {0.00, 0.00, 0.00};
float Kd[3] = {0, 13, 13};

int status = STOPPED;

float i = 0;
float start_seconds = 0;

void setup() {
  stopMotors();
  Wire.begin();
  Serial.begin(115200);  // Initialize serial communication

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  setupMPUv2();
  digitalWrite(13, LOW);
  start_seconds = millis() / 1000;
}

void loop() {
  if (!dmpReady) {
    Serial.println("MPU error");
    return;
  }

  i++;
  Serial.println(i / (millis() / 1000 - start_seconds));

  readSerialCommands();  // Read commands from Raspberry Pi
  calculateAnglesv2();
  calculateErrors();

  if (started) {
    pidController();
    applyMotorSpeeds();
  } else {
    stopMotors();
  }
}

void readSerialCommands() {
  if (Serial.available() >= 4) {
    for (int i = 0; i < 4; i++) {
      instruction[i] = Serial.parseFloat();
    }
  }
}

void setupMPUv2() {
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(gyro_offsets[0]);
  mpu.setYGyroOffset(gyro_offsets[1]);
  mpu.setZGyroOffset(gyro_offsets[2]);
  mpu.setZAccelOffset(gyro_offsets[2]);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void calculateAnglesv2() {
  while (!mpuInterrupt && fifoCount < packetSize) {
    fifoCount = mpu.getFIFOCount();
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(measures, &q, &gravity);
    measures[YAW] = measures[YAW] * 180 / M_PI;
    measures[PITCH] = measures[PITCH] * 180 / M_PI;
    measures[ROLL] = measures[ROLL] * 180 / M_PI;
  }
}

void calculateErrors() {
  errors[YAW] = instruction[YAW] - measures[YAW];
  errors[PITCH] = instruction[PITCH] - measures[PITCH];
  errors[ROLL] = instruction[ROLL] - measures[ROLL];
}
