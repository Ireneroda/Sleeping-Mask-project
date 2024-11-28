/*
   This Arduino program monitors a user's sleep position and detects snoring levels using an 
   MPU6050 accelerometer/gyroscope and a microphone sensor, respectively. A single vibrating 
   motor is used to provide corrective feedback for both functionalities.

   --- Features ---
   1. **Sleep Position Monitoring**:
      - Calculates the user's body orientation (pitch and roll) using accelerometer data.
      - Detects back sleeping, side sleeping, stomach sleeping, or unknown transitions.
      - Activates the vibrating motor when back sleeping is detected to encourage position change.

   2. **Snoring Detection**:
      - Monitors sound levels from a microphone sensor in real-time.
      - Classifies snoring as soft, moderate, or loud based on predefined thresholds.
      - Activates the vibrating motor when loud snoring is detected to prompt user adjustment.

   --- Workflow ---
   - During startup:
      1. Initializes the MPU6050 sensor and calibrates it for accurate readings.
      2. Sets up the motor pin.
   - In the main loop:
      1. Continuously monitors sleep position every 10 seconds.
      2. Detects snoring in real-time with a 500-microsecond sampling interval.
   - Feedback is provided by activating the vibrating motor for back sleeping or loud snoring.

   --- Hardware ---
   - MPU6050: Used for detecting sleep position.
   - Microphone sensor: Detects snoring based on sound levels.
   - Vibrating motor: Provides corrective feedback for both functionalities.

   This program is designed for basic sleep monitoring applications, combining both positional 
   and acoustic feedback mechanisms for improved sleep quality.
*/

#include <Wire.h>
#include <MPU6050.h>

// MPU6050 Calibration Offsets
long axOffset = 0, ayOffset = 0, azOffset = 0;
long gxOffset = 0, gyOffset = 0, gzOffset = 0;

// Pin Definitions
const int motorPin = 9; // Shared motor pin for both position and snoring-based activation

// Snoring Detection Thresholds
#define SOFT_SNORING_THRESHOLD 550
#define MODERATE_SNORING_THRESHOLD 600
#define LOUD_SNORING_THRESHOLD 700

// MPU6050 Object
MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected! Check wiring.");
    while (1);
  }
  Serial.println("MPU6050 connected!");
  calibrateMPU();
  Serial.println("MPU6050 calibration complete!");

  // Initialize motor pin
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW); // Ensure motor is off initially
}

void loop() {
  detectPosition(); // Check sleep position
  detectSnoring();  // Check snoring levels
}

// Detect Sleep Position
void detectPosition() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Apply offsets
  ax -= axOffset;
  ay -= ayOffset;
  az -= azOffset;

  // Convert to 'g' units and normalize
  float Ax = ax / 16384.0, Ay = ay / 16384.0, Az = az / 16384.0;
  float norm = sqrt(Ax * Ax + Ay * Ay + Az * Az);
  float Ax_norm = Ax / norm, Ay_norm = Ay / norm, Az_norm = Az / norm;

  // Calculate pitch and roll
  float pitch = atan2(Ax_norm, Az_norm) * 180 / PI;
  float roll = atan2(Ay_norm, Az_norm) * 180 / PI;

  // Detect sleep position
  if (abs(roll) < 15 && pitch > -20 && pitch < 20) {
    Serial.println("Position: Back Sleeping");
    activateVibratingMotor(); // Motor for back sleeping
  } else if (abs(roll) > 30 && pitch > -20 && pitch < 20) {
    Serial.println("Position: Side Sleeping");
  } else if (abs(roll) < 15 && pitch > 20) {
    Serial.println("Position: Stomach Sleeping");
  } else {
    Serial.println("Position: Unknown/Transition");
  }

  delay(10000); // Wait 10 seconds before next position check
}

// Detect Snoring
void detectSnoring() {
  static unsigned long lastSampleTime = 0;
  unsigned long currentTime = micros();

  if (currentTime - lastSampleTime >= 500) {
    lastSampleTime = currentTime;
    int sensorValue = analogRead(A0);

    // Print sensor value
    Serial.print("Microphone: ");
    Serial.println(sensorValue);

    // Detect snoring levels
    if (sensorValue >= SOFT_SNORING_THRESHOLD && sensorValue < MODERATE_SNORING_THRESHOLD) {
      Serial.println("Soft snoring detected.");
    } else if (sensorValue >= MODERATE_SNORING_THRESHOLD && sensorValue < LOUD_SNORING_THRESHOLD) {
      Serial.println("Moderate snoring detected.");
    } else if (sensorValue >= LOUD_SNORING_THRESHOLD) {
      Serial.println("Loud snoring detected!");
      activateVibratingMotor(); // Motor for loud snoring
    } else {
      Serial.println("No snoring detected.");
    }
  }
}

// Activate Vibrating Motor
void activateVibratingMotor() {
  Serial.println("Activating motor...");
  digitalWrite(motorPin, HIGH);
  delay(500); // Keep motor on for 500 ms
  digitalWrite(motorPin, LOW);
  Serial.println("Motor deactivated.");
}

// Calibrate MPU6050
void calibrateMPU() {
  int16_t ax, ay, az, gx, gy, gz;
  long axSum = 0, aySum = 0, azSum = 0, gxSum = 0, gySum = 0, gzSum = 0;

  const int samples = 1000;
  Serial.println("Calibrating MPU6050...");
  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axSum += ax;
    aySum += ay;
    azSum += az - 16384;
    gxSum += gx;
    gySum += gy;
    gzSum += gz;
    delay(2);
  }

  axOffset = axSum / samples;
  ayOffset = aySum / samples;
  azOffset = azSum / samples;
  gxOffset = gxSum / samples;
  gyOffset = gySum / samples;
  gzOffset = gzSum / samples;

  Serial.println("Calibration complete!");
}
