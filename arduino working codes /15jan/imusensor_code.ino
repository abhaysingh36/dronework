#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// MPU6050 object
Adafruit_MPU6050 mpu;
float gyroYaw, accelYaw;
sensors_event_t accelEvent, gyroEvent;
float yawAngle = 0;  // Estimated yaw angle
float alpha = 0.98;   // Filter constant (between 0 and 1)
bool throttleSafe = true;  // Global variable to track throttle safety
float P[2][2] = {{1, 0}, {0, 1}};
float deltaTime = 0.01;  // Example value for time step in seconds
float Q_angle = 0.001;  // Process noise covariance for angle
float R_angle = 0.005;  // Measurement noise covariance for angle
float yawRate = 0;  // Initialize yawRate
float rollMaxIntegral = 500.0;  // Max roll integral
float pitchMaxIntegral = 500.0; // Max pitch integral
float Q_gyro = 0.003;  // Gyro process noise covariance
unsigned long lastTime = 0;  // To store the last time for deltaTime calculation
float yawIntegral = 0.0;  // Define yawIntegral
const float ANGULAR_VELOCITY_THRESHOLD = 10.0;  // Rad/s (Adjust based on your system)
const float ACCELEROMETER_NOISE_THRESHOLD = 2.0;
float yawMaxIntegral = 1000.0; // Set an appropriate value based on your system
float ax, ay, az;
float lastYawError = 0.0; // Initialize with an appropriate value
float accelVariance = sqrt(ax * ax + ay * ay); 

// ESCs for motors
Servo esc1, esc2, esc3, esc4;
const int escPins[] = {9, 10, 11, 6};
const int minThrottle = 1000;
const int maxThrottle = 2000;

SoftwareSerial piSerial(2, 3);  // RX, TX for Raspberry Pi

float roll, pitch, yaw;  // Orientation data
float rollError, lastRollError, pitchError, lastPitchError;
float rollCorrection, pitchCorrection, yawCorrection;
float rollIntegral = 0.0, pitchIntegral = 0.0;  // Integral terms for PID
float Kp = 2.0, Ki = 0.1, Kd = 1.0;  // PID constants

int escDelay = 2000;  // Default delay in milliseconds for ESC calibration
int filterWindow = 10;  // Initial filter window size

// Moving Average Filter for noise reduction
float rollHistory[10] = {0}, pitchHistory[10] = {0};  // Initialize arrays
int rollIndex = 0, pitchIndex = 0;

int lastThrottle[4] = {1500, 1500, 1500, 1500};  // Store last throttle values
float average(float history[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += history[i];
  }
  return sum / size;
}


class ESCCalibration {
public:
  void calibrateESCs() {
    Servo* escs[] = {&esc1, &esc2, &esc3, &esc4};
    for (Servo* esc : escs) {
      esc->writeMicroseconds(maxThrottle);
      delay(escDelay);
      esc->writeMicroseconds(minThrottle);
      delay(escDelay);
    }
  }
};

void setup() {
  Serial.begin(9600);
  piSerial.begin(9600);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050");
    while (1);
  }

  esc1.attach(escPins[0]);
  esc2.attach(escPins[1]);
  esc3.attach(escPins[2]);
  esc4.attach(escPins[3]);

  Serial.println("Calibration can be performed via serial command.");
  Serial.println("Use 'setdelay <milliseconds>' to adjust ESC calibration delay.");
}

void loop() {
  readMPU6050Data();
  stabilizeDrone();

  // Process serial input from both Serial and piSerial
  processSerialInput(Serial);
  processSerialInput(piSerial);
}

void processSerialInput(SoftwareSerial& serial) {
  if (serial.available() > 0) {
    char inputBuffer[32];
    serial.readBytesUntil('\n', inputBuffer, sizeof(inputBuffer));
    inputBuffer[31] = '\0';  // Null-terminate the input
    handleSerialCommand(String(inputBuffer));
  }
}


void adjustKalmanFilterParameters() {
  // Read the gyroscope rate in degrees/s (converted from rad/s)
  float gyroRate = yawRate;  // Assuming yawRate is in degrees per second

  // Dynamically adjust Q_angle based on the gyro rate
  if (abs(gyroRate) > ANGULAR_VELOCITY_THRESHOLD) {
    Q_angle = 0.01;  // Increase when system rotates rapidly
  } else {
    Q_angle = 0.001;  // Decrease when stationary
  }

  // Calculate accelerometer variance for noise
  float accelVariance = sqrt(ax * ax + ay * ay);
  if (accelVariance > ACCELEROMETER_NOISE_THRESHOLD) {
    R_angle = 0.01;  // Increase R_angle for noisy data
  } else {
    R_angle = 0.005;  // Stable accelerometer data
  }

  // Debugging output
  Serial.print("Q_angle: "); Serial.print(Q_angle);
  Serial.print(", R_angle: "); Serial.println(R_angle);
}

bool checkThrottleSafe() {
  for (int i = 0; i < 4; ++i) {
    if (lastThrottle[i] < minThrottle) {
      throttleSafe = false;
      return false;
    }
  }
  throttleSafe = true;
  return true;
}

void calibrateESCsIfSafe() {
  if (checkThrottleSafe()) {
    ESCCalibration escCalibration;
    Serial.println("Starting ESC calibration. Please keep a safe distance.");
    piSerial.println("Starting ESC calibration. Please keep a safe distance.");
    escCalibration.calibrateESCs();
    Serial.println("ESC calibration complete.");
    piSerial.println("ESC calibration complete.");
  }
}

void killSwitch() {
  setThrottleValues(minThrottle, minThrottle, minThrottle, minThrottle);
  Serial.println("Emergency kill switch activated! Motors stopped.");
  piSerial.println("Emergency kill switch activated! Motors stopped.");
}

void readMPU6050Data() {
  sensors_event_t accelEvent, gyroEvent;
  mpu.getEvent(&accelEvent, &gyroEvent, NULL);


  float gyroRate = gyroEvent.gyro.z / 131.0;
  float accelYaw = atan2(accelEvent.acceleration.y, accelEvent.acceleration.x) * 180 / PI;

  // Kalman filter prediction step
  yawAngle += gyroRate * deltaTime;
  P[0][0] += deltaTime * (P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= deltaTime * P[1][1];
  P[1][0] -= deltaTime * P[1][1];
  P[1][1] += Q_gyro * deltaTime;

  // Kalman filter update step
  float S = P[0][0] + R_angle;
  float K_0 = P[0][0] / S;
  float K_1 = P[1][0] / S;
  float y = accelYaw - yawAngle;  // Innovation
  yawAngle += K_0 * y;
  yawRate += K_1 * y;

  // Update error covariance
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K_0 * P00_temp;
  P[0][1] -= K_0 * P01_temp;
  P[1][0] -= K_1 * P00_temp;
  P[1][1] -= K_1 * P01_temp;
}

void stabilizeDrone() {
  rollError = roll;  // Current error for roll
  pitchError = pitch;  // Current error for pitch

  // Prevent integral windup
  rollIntegral = constrain(rollIntegral, -rollMaxIntegral, rollMaxIntegral);
  pitchIntegral = constrain(pitchIntegral, -pitchMaxIntegral, pitchMaxIntegral);

  // PID control for roll and pitch
  rollCorrection = Kp * rollError + Ki * rollIntegral + Kd * (rollError - lastRollError);
  pitchCorrection = Kp * pitchError + Ki * pitchIntegral + Kd * (pitchError - lastPitchError);

  // Store the current errors for the next loop
  lastRollError = rollError;
  lastPitchError = pitchError;

  // Yaw stabilization
  float yawError = yawAngle;
  yawIntegral += yawError * deltaTime;
  yawIntegral = constrain(yawIntegral, -yawMaxIntegral, yawMaxIntegral);

  yawCorrection = Kp * yawError + Ki * yawIntegral + Kd * (yawError - lastYawError);

  // Store the previous yaw error for the next loop
  lastYawError = yawError;

  // Apply throttle corrections for all motors
  int throttleBase = 1500;  // Mid-range throttle

  setThrottleValues(
    throttleBase + rollCorrection - pitchCorrection - yawCorrection,  // Motor 1
    throttleBase - rollCorrection - pitchCorrection + yawCorrection,  // Motor 2
    throttleBase + rollCorrection + pitchCorrection + yawCorrection,  // Motor 3
    throttleBase - rollCorrection + pitchCorrection - yawCorrection   // Motor 4
  );
}







void handleSerialCommand(String command) {
  command.trim();
  if (command == "kill") {
    killSwitch();  // Call kill switch
  } else if (command == "calibrate") {
    calibrateESCsIfSafe();  // Use safe calibration method
  } else if (command.startsWith("setdelay")) {
    int newDelay = command.substring(8).toInt();
    if (newDelay > 0) {
      escDelay = newDelay;
      Serial.print("ESC calibration delay set to: ");
      Serial.println(escDelay);
      piSerial.print("ESC calibration delay set to: ");
      piSerial.println(escDelay);
    } else {
      Serial.println("Invalid delay value. Enter a positive integer.");
      piSerial.println("Invalid delay value. Enter a positive integer.");
    }
  } else if (command.startsWith("setpid")) {
    setPIDParameters(command);
  } else if (command.startsWith("setfilterwindow")) {
    int newFilterWindow = command.substring(16).toInt();
    if (newFilterWindow > 1 && newFilterWindow <= 10) {
      filterWindow = newFilterWindow;
      Serial.print("Filter window size set to: ");
      Serial.println(filterWindow);
      piSerial.print("Filter window size set to: ");
      piSerial.println(filterWindow);
    } else {
      Serial.println("Invalid filter window size. Enter a value between 2 and 10.");
      piSerial.println("Invalid filter window size. Enter a value between 2 and 10.");
    }
  } else {
    processThrottleInput(command);
  }
}


void setPIDParameters(const String& command) {
  int firstSpace = command.indexOf(' ', 7);
  int secondSpace = command.indexOf(' ', firstSpace + 1);

  float newKp = command.substring(7, firstSpace).toFloat();
  float newKi = command.substring(firstSpace + 1, secondSpace).toFloat();
  float newKd = command.substring(secondSpace + 1).toFloat();

  if (newKp > 0 && newKi >= 0 && newKd >= 0) {
    Kp = newKp;
    Ki = newKi;
    Kd = newKd;
    Serial.print("PID values updated to: ");
    Serial.print("Kp: "); Serial.print(Kp);
    Serial.print(", Ki: "); Serial.print(Ki);
    Serial.print(", Kd: "); Serial.println(Kd);
    piSerial.print("PID values updated to: ");
    piSerial.print("Kp: "); piSerial.print(Kp);
    piSerial.print(", Ki: "); piSerial.print(Ki);
    piSerial.print(", Kd: "); piSerial.println(Kd);
  } else {
    Serial.println("Invalid PID values. Enter positive values.");
    piSerial.println("Invalid PID values. Enter positive values.");
  }
}

void processThrottleInput(String input) {
  int throttleValues[4];
  if (parseThrottleInput(input, throttleValues)) {
    setThrottleValues(throttleValues[0], throttleValues[1], throttleValues[2], throttleValues[3]);
    Serial.print("Throttle set to: ");
    Serial.println(input);
    piSerial.print("Throttle set to: ");
    piSerial.println(input);
  } else {
    Serial.println("Invalid input. Enter four values between 1000 and 2000 separated by commas.");
    piSerial.println("Invalid input. Enter four values between 1000 and 2000 separated by commas.");
  }
}




bool parseThrottleInput(const String& input, int throttleValues[4]) {
  int index = 0;
  int lastIndex = 0;
  for (int i = 0; i < 4; ++i) {
    int commaIndex = input.indexOf(',', lastIndex);
    if (i < 3 && commaIndex == -1) return false;
    if (i == 3) commaIndex = input.length();

    String value = input.substring(lastIndex, commaIndex);
    int throttle = value.toInt();
    if (throttle >= minThrottle && throttle <= maxThrottle) {
      throttleValues[i] = throttle;
    } else {
      return false;
    }
    lastIndex = commaIndex + 1;
  }
  return true;
}
void applyMovingAverageFilter() {
    rollHistory[rollIndex] = roll;
    pitchHistory[pitchIndex] = pitch;

    rollIndex = (rollIndex + 1) % filterWindow;
    pitchIndex = (pitchIndex + 1) % filterWindow;

    roll = average(rollHistory, filterWindow);
    pitch = average(pitchHistory, filterWindow);
}


void setThrottleValues(int m1, int m2, int m3, int m4) {
  // Applying throttle smoothing
  esc1.writeMicroseconds(smoothThrottle(lastThrottle[0], m1));
  esc2.writeMicroseconds(smoothThrottle(lastThrottle[1], m2));
  esc3.writeMicroseconds(smoothThrottle(lastThrottle[2], m3));
  esc4.writeMicroseconds(smoothThrottle(lastThrottle[3], m4));

  // Store last throttle values for smooth transition
  lastThrottle[0] = m1;
  lastThrottle[1] = m2;
  lastThrottle[2] = m3;
  lastThrottle[3] = m4;
}

int smoothThrottle(int currentThrottle, int targetThrottle) {
    const int MAX_THROTTLE_STEP = 30;  // More subtle steps
    int delta = targetThrottle - currentThrottle;
    delta = constrain(delta, -MAX_THROTTLE_STEP, MAX_THROTTLE_STEP);
    return currentThrottle + delta;
}