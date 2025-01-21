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
