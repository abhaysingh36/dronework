#include <Servo.h>  // Include the Servo library for generating PWM signals

Servo esc;  // Create a servo object to control the ESC
const int escPin = 9;  // PWM signal output pin

const int minThrottle = 1000;  // Minimum pulse width in microseconds
const int maxThrottle = 2000;  // Maximum pulse width in microseconds
int throttleValue = minThrottle;  // Initial throttle value

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  esc.attach(escPin);  // Attach the ESC to the defined pin

  // ESC calibration sequence
  Serial.println("Calibrating ESC...");
  esc.writeMicroseconds(maxThrottle);  // Set maximum throttle for calibration
  delay(2000);  // Wait for ESC to register
  esc.writeMicroseconds(minThrottle);  // Set minimum throttle
  delay(2000);  // Calibration complete
  Serial.println("Calibration complete. Enter throttle (1000-2000):");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read serial input
    int throttle = input.toInt();  // Convert input to an integer

    // Validate input range
    if (throttle >= minThrottle && throttle <= maxThrottle) {
      throttleValue = throttle;
      esc.writeMicroseconds(throttleValue);  // Send throttle value to ESC
      Serial.print("Throttle set to: ");
      Serial.println(throttleValue);
    } else {
      Serial.println("Invalid throttle. Enter a value between 1000 and 2000.");
    }
  }
}


## code 2 

#include <Servo.h>  // Include the Servo library for generating PWM signals

Servo esc1, esc2, esc3, esc4;  // Create servo objects to control the ESCs
const int escPins[] = {9, 10, 11, 6};  // PWM signal output pins

const int minThrottle = 1000;  // Minimum pulse width in microseconds
const int maxThrottle = 2000;  // Maximum pulse width in microseconds

void setup() {
  Serial.begin(9600);  // Initialize serial communication

  // Attach each ESC to the corresponding pin
  esc1.attach(escPins[0]);
  esc2.attach(escPins[1]);
  esc3.attach(escPins[2]);
  esc4.attach(escPins[3]);

  Serial.println("Calibrating ESCs...");
  calibrateESCs();  // Run calibration for all ESCs
  Serial.println("Calibration complete. Enter throttle (1000-2000):");
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

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read serial input
    processThrottleInput(input);
  }
}

void processThrottleInput(String input) {
  int throttle = input.toInt();  // Convert input to an integer

  // Validate input range
  if (throttle >= minThrottle && throttle <= maxThrottle) {
    setThrottleValues(throttle);
    Serial.print("Throttle set to: ");
    Serial.println(throttle);
  } else {
    Serial.println("Invalid throttle. Enter a value between 1000 and 2000.");
  }
}

void setThrottleValues(int throttle) {
  esc1.writeMicroseconds(throttle);
  esc2.writeMicroseconds(throttle);
  esc3.writeMicroseconds(throttle);
  esc4.writeMicroseconds(throttle);
}



