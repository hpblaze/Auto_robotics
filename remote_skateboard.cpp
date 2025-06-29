// Include necessary libraries
#include <Wire.h>           // For I2C communication with ADXL345
#include <Servo.h>          // For controlling the servo motor (analog speed display)
#include <Stepper.h>        // For controlling the stepper motor

// Define pin connections
const int motorPin = 3;               // DC motor speed control (PWM)
const int dataPin = 11;               // Data pin for shift register [DS](7-segment display)
const int clockPin = 10;              // Clock pin for shift register [SHCP]
const int latchPin = 9;               // Latch pin for shift register [STCP]
const int servoPin = 7;               // Servo motor pin (analog speedometer)
const int trigPin = 12, echoPin = 13; // Ultrasonic sensor pins
const int ldrPin = A4;                // Photoresistor (light sensor)
const int potLedPin = A5;             // Potentiometer for LED control
const int joystickX = A0, joystickY = A1; // Joystick analog inputs
const int joystickBtn = 8;            // Joystick button pin for mode toggle
const int step1 = 6;                  // Pins connected to stepper motor via H-bridge
const int step2 = 4;
const int step3 = 5;
const int step4 = 2;

Servo speedServo;                     // Servo object for analog speed display

const int stepsPerRev = 2048;
Stepper myStepper = Stepper(stepsPerRev, step1, step2, step3, step4);

int ADXL345 = 0x53;                   // I2C address of the ADXL345 accelerometer
float accelX = 0, accelY = 0, accelZ = 0;         // initial Tilt values from accelerometer
float acc_X_raw, acc_Y_raw, acc_Z_raw;// Output
float prevZ = 0, currentZ = 0;        // For initial fall detection 

// Control flags and speed variables
bool droneMode = false;               // initial Drone mode flag
float speed = 0;                      // Current speed of the skateboard
int lastJoyBtnState = 1;              // For detecting joystick button press
unsigned long lastDebounceTime = 0;   // Debounce timer
unsigned long debounceDelay = 50; // Debounce delay in ms
const float maxSpeed = 9.0, minSpeed = 0.0; // Speed limits
unsigned long lastUpdateTime = 0;     // For updating speed periodically
const unsigned long updateInterval = 100; // Update interval in ms
unsigned long lastFall = 0;           // Last fall detection time
const unsigned long fallCooldown = 1000; // Cooldown to prevent repeated fall detection
const float fallDropThreshold = 0.1;  // Z-axis drop threshold for fall detection
//const int LDR_DARK_THRESHOLD = 800; // Adjust this value (0-1023) to set when the LED turns on. Higher value = needs to be darker.


// 7-segment display digit patterns (Common Cathode)
// Format: 0-9 => Q0-Q6 (segments), Q7 = LED control
byte digits[10] = {
  B00111111, B00000110, B01011011, B01001111, B01100110,
  B01101101, B01111101, B00000111, B01111111, B01101111
};

void setup() {
  Serial.begin(9600);              // Start serial communication
  Wire.begin();                    // Initialize I2C

  // Initialize ADXL345 accelerometer
  Wire.beginTransmission(ADXL345); // Start communicating with the device
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  //Enable mesurement
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable
  Wire.endTransmission();
  delay(100);

  // Set pin modes
  pinMode(motorPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(joystickBtn, INPUT);
  digitalWrite(joystickBtn, HIGH);
  pinMode(potLedPin, INPUT);
  pinMode(ldrPin, INPUT);

  //analogWrite(motorPin, 0);        // Start motor at 0 speed
  speedServo.attach(servoPin);     // Attach servo motor
  speedServo.write(0);             // Initialize at 0 degrees
}

void loop() {
  float distance = readUltrasonic(); // Get obstacle distance
  checkDroneToggle();               // Check for drone mode toggle
  readAccel();                      // Read accelerometer values

  // Manual mode: control via X-axis tilt
  if (!droneMode) {
    updateSpeedOverTime();         // Adjust speed using accelerometer tilt
    if (isFallDetected()) {        // Check for fall
      lastFall = millis();
      speed = 0;                   // Stop motor
      applyMotorSpeed(speed);
      Serial.println("Fall detected! Stopping.");
    }
  } 
  // Drone mode: control via joystick
  else {
    updateSpeedFromJoystick();

    int joyX = analogRead(joystickX);
    if (joyX > 600){
      myStepper.setSpeed(5);
      myStepper.step(50);
    }
    else if (joyX < 400) {
      myStepper.setSpeed(5);
      myStepper.step(-50);
    }
  }

  // Limit speed
  speed = constrain(speed, 0, 9);
  applyObstacleBraking(distance);  // Slow down if obstacle detected

  // LED brightness control
  int ldrVal = analogRead(ldrPin);
  int potVal = analogRead(potLedPin);
  updateDisplayAndLED((int)speed, potVal, ldrVal); // Display speed on 7-segment and LED feedback

  //update7Segment((int)speed);      // Display speed on 7-segment
  speedServo.write(map(speed, 0, 9, 0, 180)); // Move servo accordingly
  applyMotorSpeed(speed); // Set motor speed
  //displaySpeedWithLED((int)speed, ledOn); // LED feedback

  Serial.println("Mode: ");Serial.print(droneMode ? "Drone" : "Manual");
  Serial.print("accelX: "); Serial.print(accelX, 3);
  Serial.print(" | speed: "); Serial.print(speed, 2);
  Serial.print(" | zHeight: "); Serial.print(currentZ, 3);
  Serial.print(" | obstacleDist: "); Serial.print(distance, 1);
  Serial.print(" | LDR: "); Serial.print(ldrVal);
  Serial.print(" | POT: "); Serial.print(potVal);
  Serial.println("-----------------------------------------------------");

  delay(100);
}

// Function to read accelerometer values
void readAccel() {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis
  acc_X_raw = (Wire.read() | (Wire.read() << 8)); // X-axis raw value
  acc_Y_raw = (Wire.read() | (Wire.read() << 8));  // Y-axis raw value
  acc_Z_raw = (Wire.read() | (Wire.read() << 8));  // Z-axis raw value
  accelX = acc_X_raw / 256; // For a range of +-2g, we need to divide the raw value
  prevZ = currentZ;
  currentZ = acc_Z_raw / 256;
}

// Detects a fall based on Z-axis change
bool isFallDetected() {
  float drop = prevZ - currentZ;
  return (drop > fallDropThreshold) && (millis() - lastFall > fallCooldown);
}

// Adjust speed based on accelerometer tilt
void updateSpeedOverTime() {
  if (millis() - lastUpdateTime >= updateInterval) {
    if (abs(accelX) > 0.05) {
      speed += accelX > 0 ? 0.5 : -0.5;
      lastUpdateTime = millis();
    }
  }
}

// Time-based joystick speed control
void updateSpeedFromJoystick() {
  static unsigned long lastJoyUpdate = 0;
  if (millis() - lastJoyUpdate >= updateInterval) {
    int joyY = analogRead(joystickY);
    if (joyY > 600) speed += 0.5;
    else if (joyY < 400) speed -= 0.5;
    lastJoyUpdate = millis();
  }
}

// Measure distance using ultrasonic sensor
float readUltrasonic() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

// Apply PWM to motor
void applyMotorSpeed(int speedVal) {
  analogWrite(motorPin, map(speedVal, 0, 9, 0, 255));
}

// Slow down if obstacle detected
void applyObstacleBraking(float distance) {
  if (distance < 30.0 && distance > 0.0) {
    float factor = 1.0 - (distance / 30.0);
    speed -= factor * 1.0;
  }
}

// Display Speed on 7-Segment and Control LED
void updateDisplayAndLED(int speedVal, int potVal, int ldrVal) {
  // LED logic: use LDR unless pot override is active
  bool ledOn;
  // Check if potentiometer is being used to override LDR
  if (potVal < 500) {
    int ambientBrightness = map(ldrVal, 0, 1023, 255, 0);
    ledOn = (ambientBrightness > 127);
  } else {
    int manualBrightness = map(potVal, 0, 1023, 0, 255);
    ledOn = (manualBrightness > 127);
  }

  // Get the 7-segment pattern for the current speed
  byte segPattern = digits[constrain(speedVal, 0, 9)];

  // Set the 8th bit (Q7) high if the LED should be on
  if (ledOn) segPattern |= B10000000; // Turn on LED (Q7)
  else       segPattern &= ~B10000000; // Turn off LED

  // Send the final byte to the shift register
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, segPattern);
  digitalWrite(latchPin, HIGH);
}

// Toggle between drone and manual mode
void checkDroneToggle() {
  int reading = digitalRead(joystickBtn);

  if (reading == LOW && lastJoyBtnState == HIGH && (millis()-lastDebounceTime) > debounceDelay){
    droneMode = !droneMode;
    lastDebounceTime = millis();
  }
  lastJoyBtnState = reading;
}

