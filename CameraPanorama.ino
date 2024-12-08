#include <Servo.h>
#include <AccelStepper.h>

// Joystick pins
const int potPinH = A1;   // Horizontal potentiometer connected to A1
const int potPinV = A0;   // Vertical potentiometer connected to A0

// Stepper motor pins (using CNC Shield pin assignments)
const int stepPin = 3;    // STEP pin for A4988
const int dirPin = 6;     // DIR pin for A4988
const int enable = 8;     // EN pin for A4988 (active LOW)

const int button = 4;     // Button for reset (optional)

// Create Servo object
Servo myservo;

// Create AccelStepper object
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// Variables for servo control
int servoAngle = 90;             // Initial servo angle
unsigned long lastServoUpdate = 0;
const int servoMinAngle = 60;
const int servoMaxAngle = 135;
const int deadzone = 13;         // Deadzone around center position
unsigned long servoUpdateInterval = 20; // Initial interval in milliseconds
const unsigned long servoMinInterval = 20;  // Fastest servo update interval
const unsigned long servoMaxInterval = 200; // Slowest servo update interval

void setup() {
  // Set up the pins
  myservo.attach(9);
  pinMode(enable, OUTPUT);
  digitalWrite(enable, LOW); // Enable the stepper driver (active LOW)
  pinMode(button, INPUT_PULLUP); // If using the button as input

  // Initialize the stepper motor
  stepper.setMaxSpeed(150);       // Set a reasonable max speed (adjust as needed)
  stepper.setAcceleration(150);   // Set acceleration (adjust as needed)

  // Initialize Serial for debugging if needed
  // Serial.begin(9600);
}

void loop() {
  // Read the joystick values
  int potValueH = analogRead(potPinH);
  int potValueV = analogRead(potPinV);

  // ---- Stepper Motor Control ----
  int deviationH = potValueH - 512;  // Center position is around 512

  if(potValueH > 495 & potValueH < 525){
    digitalWrite(enable, HIGH); // Disable the motor
  } else {
    digitalWrite(enable, LOW); // Enable the motor
  }

  if (abs(deviationH) > deadzone) {
    digitalWrite(enable, LOW); // Enable the motor

    // Map joystick input to speed
    float speed = map(deviationH, -511, 511, -stepper.maxSpeed(), stepper.maxSpeed());
    stepper.setSpeed(speed);
  } else {
    // Joystick is in deadzone; stop the motor
    stepper.setSpeed(0);
  }

  // Run the motor
  stepper.runSpeed();

  // ---- Servo Control ----
  int deviationV = potValueV - 512;

  if (abs(deviationV) > deadzone) {
    // Map the deviation to servo update interval
    unsigned long servoUpdateInterval = map(abs(deviationV), deadzone, 511, servoMaxInterval, servoMinInterval);

    if (millis() - lastServoUpdate >= servoUpdateInterval) {
      lastServoUpdate = millis();

      if (deviationV > 0) {
        // Move servo forward
        servoAngle += 1;
        if (servoAngle > servoMaxAngle) servoAngle = servoMaxAngle;
      } else {
        // Move servo backward
        servoAngle -= 1;
        if (servoAngle < servoMinAngle) servoAngle = servoMinAngle;
      }
      // Write the new angle to the servo
      myservo.write(servoAngle);
    }
  }
  // If within deadzone, do not update the servo angle
}
