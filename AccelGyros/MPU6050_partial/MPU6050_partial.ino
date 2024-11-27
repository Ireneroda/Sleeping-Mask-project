#include <Wire.h>
#include <MPU6050.h>


// Initialize MPU6050 object
MPU6050 mpu;

// Calibration offsets for accelerometer and gyroscope
long axOffset = 0, ayOffset = 0, azOffset = 0;
long gxOffset = 0, gyOffset = 0, gzOffset = 0;

// Pin for vibrating motor
const int motorPin = 9;

void setup() {
  Serial.begin(9600); // Start Serial communication
  Wire.begin();       // Initialize I2C communication

  // Initialize MPU6050
  mpu.initialize();

  // Check if MPU6050 is connected
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected! Please check the connections.");
    while (1); // Halt execution if MPU6050 is not found
  }
  Serial.println("MPU6050 connected!");

  // Perform calibration to get offsets
  calibrateMPU();
  Serial.println("Calibration complete!");

  // Initialize motor pin
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW); // Ensure motor is off initially
}

void loop() {
  
// Variables to store raw sensor data
  int16_t ax, ay, az, gx, gy, gz;

  // Read raw accelerometer and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Apply calibration offsets
  ax -= axOffset;
  ay -= ayOffset;
  az -= azOffset;
  gx -= gxOffset;
  gy -= gyOffset;
  gz -= gzOffset;

  // Convert accelerometer data to 'g' units (1g = 16384 LSB)
  float Ax = ax / 16384.0;
  float Ay = ay / 16384.0;
  float Az = az / 16384.0;

  // Normalize accelerometer data for pitch/roll calculation
  float norm = sqrt(Ax * Ax + Ay * Ay + Az * Az);
  float Ax_norm = Ax / norm;
  float Ay_norm = Ay / norm;
  float Az_norm = Az / norm;

  // Calculate pitch and roll (in radians)
  float pitch = atan2(Ax_norm, Az_norm); // Up/down tilt
  float roll = atan2(Ay_norm, Az_norm);  // Side-to-side tilt

  // Convert pitch and roll to degrees
  pitch = pitch * 180 / PI;
  roll = roll * 180 / PI;

  // Output sensor data and orientation
  
  Serial.print(Ax); Serial.print(", ");
  Serial.print(Ay); Serial.print(", ");
  Serial.print(Az); Serial.print(" , ");
  Serial.print(pitch); Serial.print(" , ");
  Serial.print(roll); Serial.println(" , ");

  // Detect sleep position and take actions
  if (abs(roll) < 15 && pitch > -20 && pitch < 20) {
    Serial.println("Back Sleeping");
  } else if (abs(roll) > 30 && pitch > -20 && pitch < 20) {
    Serial.println("Side Sleeping");
  } else if (abs(roll) < 15 && pitch > 20) {
    Serial.println("Stomach Sleeping");
  } else {
    Serial.println("Unknown/Transition");
  }

  // Wait 10 seconds before the next reading
  delay(10000);
}

// Function to activate vibrating motor
void activateVibratingMotor() {
  
  digitalWrite(motorPin, HIGH); // Turn on the motor
  delay(500);                   // Keep the motor on for 500 ms
  digitalWrite(motorPin, LOW);  // Turn off the motor
  
}

// Function to calibrate MPU6050 and calculate offsets
void calibrateMPU() {
  int16_t ax, ay, az, gx, gy, gz;
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  const int samples = 1000; // Number of samples for averaging

  Serial.println("Calibrating MPU6050...");
  for (int i = 0; i < samples; i++) {
    // Read raw sensor data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Accumulate data for averaging
    axSum += ax;
    aySum += ay;
    azSum += az - 16384; // Subtract 1g (gravity)
    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    delay(2); // Short delay between readings
  }

  // Calculate averages to determine offsets
  axOffset = axSum / samples;
  ayOffset = aySum / samples;
  azOffset = azSum / samples;
  gxOffset = gxSum / samples;
  gyOffset = gySum / samples;
  gzOffset = gzSum / samples;

  // Output calculated offsets
  Serial.println("Offsets calculated:");
  Serial.print("Accel Offsets: ");
  Serial.print(axOffset); Serial.print(", ");
  Serial.print(ayOffset); Serial.print(", ");
  Serial.println(azOffset);
  Serial.print("Gyro Offsets: ");
  Serial.print(gxOffset); Serial.print(", ");
  Serial.print(gyOffset); Serial.print(", ");
  Serial.println(gzOffset);
}
