#define SOFT_SNORING_THRESHOLD 550    // Soft snoring threshold (lower bound)
#define MODERATE_SNORING_THRESHOLD 600 // Moderate snoring threshold (lower bound)
#define LOUD_SNORING_THRESHOLD 700     // Loud snoring threshold (upper bound)
#define MOTOR_PIN 5                   // Pin where the vibrating motor is connected

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(MOTOR_PIN, OUTPUT);  // Set motor pin as output
  digitalWrite(MOTOR_PIN, LOW); // Ensure motor is off initially
}

// Function to activate the vibrating motor
void activateVibratingMotor() {
  digitalWrite(MOTOR_PIN, HIGH);  // Turn on the motor
  delay(1000);                    // Keep motor on for 1 second (you can adjust this)
  digitalWrite(MOTOR_PIN, LOW);   // Turn off the motor
}


// Microphone detect snoring at fs=2000 KHz / time delay = 0.5 (ms)
void loop() {
  static unsigned long lastSampleTime = 0; // Keeps track of the last sample time
  unsigned long currentTime = micros();   // Get the current time in microseconds

  // Check if 0.5 milliseconds (500 microseconds) have passed
  if (currentTime - lastSampleTime >= 500) {
    lastSampleTime = currentTime;         // Update the last sample time
    int sensorValue = analogRead(A0);     // Read the microphone output
    Serial.println(sensorValue);     // Print the value to Serial Monitor

    //Classify snoring
    switch (sensorValue) {
      case 550 ... 599:
        Serial.println("Soft snoring detected (light snoring).");
        break;

      case 600 ... 699:
        Serial.println("Moderate snoring detected.");
        break;

      case 700 ... 1023:  // 1023 is the maximum possible value for an analogRead
        Serial.println("Loud snoring detected!");
        activateVibratingMotor(); // Activate vibrating motor for loud snoring
        break;

      default:
        Serial.println("No snoring detected.");
        break;
    }
  }

  
}
