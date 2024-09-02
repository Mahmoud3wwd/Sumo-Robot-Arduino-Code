#include <QTRSensors.h>

// Create an instance of the QTRSensors class
QTRSensors qtr;
unsigned int qtrValues[4]; // Array to hold QTR sensor values

// Define QTR sensor pins
const unsigned char qtrPins[4] = {A1, A2, A0, A3}; // Analog pins A0, A1, A2, and A3

// Define infrared sensor pins
const unsigned char irPins[5] = {7, 8, 4, 12, 2}; // Digital pins 7, 8, 4, 12, and 2
unsigned int irValues[5]; // Array to hold E10 infrared sensor values

// Named variables for infrared sensors
const int Front = 0;
const int FrontLeft = 1;
const int LeftSide = 2; 
const int FrontRight = 3;
const int RightSide = 4;

// Motor control pins for Driver 1 (Motor 1 and Motor 2)
const int motor1_pwm = 3;
const int motor1_enableL = 5;
const int motor1_enableR = 6;

// Motor control pins for Driver 2 (Motor 3 and Motor 4)
const int motor2_pwm = 9;
const int motor2_enableL = 10;
const int motor2_enableR = 11;

// Define Arduino reciever pin
const int AR = 1;

// Define QTR sensor position indices
const int UPPER_LEFT = 0;
const int UPPER_RIGHT = 1;
const int LOWER_LEFT = 2;
const int LOWER_RIGHT = 3;

void setup() {
  Serial.begin(9600);

  // Set motor control pins as outputs
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_enableL, OUTPUT);
  pinMode(motor1_enableR, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor2_enableL, OUTPUT);
  pinMode(motor2_enableR, OUTPUT);

  // Initialize QTRSensors with the pin configuration
  qtr.setTypeAnalog(); // Set type to analog to handle digital sensors as well
  qtr.setSensorPins(qtrPins, 4);

  // Set IR sensor pins as inputs
  for (int i = 0; i < 5; i++) {
    pinMode(irPins[i], INPUT);
  }


  // Initialize serial communication for debugging
  Serial.begin(9600);
  delay(5000); 
}

void loop() {
 
    // Read the QTR sensor values
    qtr.read(qtrValues);

    // Read the IR sensor values
    bool anyIRDetected = false; // Variable to check if any IR sensor detects something
    for (int i = 0; i < 5; i++) {
      irValues[i] = digitalRead(irPins[i]);
      if (irValues[i] == LOW) {
        anyIRDetected = true;
      }
    }

    // Determine if any QTR sensor detects white (edge)
    bool edgeDetected = false;
    for (int i = 0; i < 4; i++) {
      if (qtrValues[i] < 500) { // Adjust threshold as needed
        edgeDetected = true;
      }
    }

    if (edgeDetected) {
      // Handle edge detection
      handleEdgeDetection();
    } 
    else if (anyIRDetected) {
      // Handle IR sensor detections
      handleIRDetection();
    } 
    else {
      // Default behavior if no sensors detect obstacles
     //randomSearch();
     tornadoSearch();
    }

    delay(100);  // Adjust as needed
  
}

// Function to handle edge detection
void handleEdgeDetection() {
  if (qtrValues[UPPER_LEFT] < 500) {
    // Upper left edge detected, reverse and turn right
    stopMotors();
    delay(10);  // Brief stop
    moveBackward();
    delay(10);  // Adjust as needed
    turnRight();
    delay(10);  // Adjust as needed
  } 
  else if (qtrValues[UPPER_RIGHT] < 500) {
    // Upper right edge detected, reverse and turn left
    stopMotors();
    delay(10);  // Brief stop
    moveBackward();
    delay(10);  // Adjust as needed
    turnLeft();
    delay(10);  // Adjust as needed
  } 
  else if (qtrValues[LOWER_LEFT] < 500) {
    // Lower left edge detected, move forward and turn left
    stopMotors();
    delay(10);  // Brief stop
    moveForward();
    delay(10);  // Adjust as needed
    turnLeft();
    delay(10);  // Adjust as needed
  } 
  else if (qtrValues[LOWER_RIGHT] < 500) {
    // Lower right edge detected, move forward and turn right
    stopMotors();
    delay(10);  // Brief stop
    moveForward();
    delay(10);  // Adjust as needed
    turnRight();
    delay(10);  // Adjust as needed
  }
  
}

// Function to handle IR sensor detections
void handleIRDetection() {
  if (irValues[Front] == LOW && irValues[FrontLeft] == LOW && irValues[FrontRight] == LOW) {
    // Front, Left Front, and Right Front detect the opponent
    //circleAndAttack();
    attackOpponent();
  } 
  else if (irValues[Front] == LOW && irValues[FrontLeft] == LOW) {
    // Front and Left Front detect the opponent
    moveForwardWithLeaning(true);  // Lean slightly to the left
  } 
  else if (irValues[Front] == LOW && irValues[FrontRight] == LOW) {
    // Front and Right Front detect the opponent
    moveForwardWithLeaning(false); // Lean slightly to the right
  } 
  else if (irValues[Front] == LOW) {
    // Only Front detects the opponent
    moveForward();
  } 
  else if (irValues[FrontLeft] == LOW) {
    // Only Left Front detects the opponent
    turnLeft(); // Turn 45 degrees to the left
  } 
  else if (irValues[FrontRight] == LOW) {
    // Only Right Front detects the opponent
    turnRight(); // Turn 45 degrees to the right
  } 
  else if (irValues[LeftSide] == LOW) {
    // Only Left Side detects an obstacle
    turnLeft();
  } 
  else if (irValues[RightSide] == LOW) {
    // Only Right Side detects an obstacle
    turnRight();
  }
}

// Function to execute the circling and attacking strategy
void circleAndAttack() {
  int turnDuration = 30; // Duration to turn around the opponent
  int attackDelay = 250;   // Delay before attacking after circling

  // Turn around the opponent
  turnRight(); // Start turning around
  delay(turnDuration); // Adjust duration based on your robot's turning speed
  stopMotors(); // Stop turning

  // Move forward for a short time to reposition for attack
  moveForward();
  delay(attackDelay);
  stopMotors();

  // Perform the attack
  attackOpponent();

  // Return to circling or random search
}

// Function to execute attack strategy
void attackOpponent() {
  // Add your attack logic here
  // Example: move forward and/or perform a maneuver
   digitalWrite(motor1_enableL, HIGH);
  digitalWrite(motor1_enableR, LOW);
  digitalWrite(motor2_enableL, HIGH);
  digitalWrite(motor2_enableR, LOW);

  analogWrite(motor1_pwm, 255);    // Set speed for Motor 1
  analogWrite(motor2_pwm, 255);    // Set speed for Motor 2
  delay(200);  // Adjust timing for attack duration
    for (int i = 0; i < 5; i++) {
      irValues[i] = digitalRead(irPins[i]); }
      if(irValues[Front] == LOW && irValues[FrontLeft] == LOW && irValues[FrontRight] == LOW){
        attackOpponent();
      }

  stopMotors();
  delay(100);  // Pause before next action
}

// Function to execute random search strategy
void randomSearch() {
  int randomAction = random(0, 3); // Generate a random number between 0 and 2

  switch (randomAction) {
    case 0:
      moveForward();
      break;
    case 1:
      turnLeft();
      break;
    case 2:
      turnRight();
      break;
  }

  delay(200); // Adjust delay as needed for the duration of each action
}

// Function to stop all motors
void stopMotors() {
  digitalWrite(motor1_enableL, LOW);
  digitalWrite(motor1_enableR, LOW);
  digitalWrite(motor2_enableL, LOW);
  digitalWrite(motor2_enableR, LOW);
}

// Function to move forward
void moveForward() {
  digitalWrite(motor1_enableL, HIGH);
  digitalWrite(motor1_enableR, LOW);
  digitalWrite(motor2_enableL, HIGH);
  digitalWrite(motor2_enableR, LOW);

  analogWrite(motor1_pwm, 150);    // Set speed for Motor 1
  analogWrite(motor2_pwm, 150);    // Set speed for Motor 2
}

// Function to move backward
void moveBackward() {
  digitalWrite(motor1_enableL, LOW);
  digitalWrite(motor1_enableR, HIGH);
  digitalWrite(motor2_enableL, LOW);
  digitalWrite(motor2_enableR, HIGH);

  analogWrite(motor1_pwm, 150);    // Set speed for Motor 1
  analogWrite(motor2_pwm, 150);    // Set speed for Motor 2
}

// Function to turn left
void turnLeft() {
  digitalWrite(motor1_enableL, LOW);
  digitalWrite(motor1_enableR, HIGH);
  digitalWrite(motor2_enableL, HIGH);
  digitalWrite(motor2_enableR, LOW);

  analogWrite(motor1_pwm, 90);    // Set speed for Motor 1
  analogWrite(motor2_pwm, 90);    // Set speed for Motor 2
}

// Function to turn right
void turnRight() {
  digitalWrite(motor1_enableL, HIGH);
  digitalWrite(motor1_enableR, LOW);
  digitalWrite(motor2_enableL, LOW);
  digitalWrite(motor2_enableR, HIGH);

  analogWrite(motor1_pwm, 90);    // Set speed for Motor 1
  analogWrite(motor2_pwm, 90);    // Set speed for Motor 2
}

// Function to move forward with slight leaning
void moveForwardWithLeaning(bool leanLeft) {
  digitalWrite(motor1_enableL, HIGH);
  digitalWrite(motor1_enableR, LOW);
  digitalWrite(motor2_enableL, HIGH);
  digitalWrite(motor2_enableR, LOW);

  if (leanLeft) {
    analogWrite(motor1_pwm, 110);    // Reduced speed for Motor 1
    analogWrite(motor2_pwm, 80);    // Full speed for Motor 2
  } else {
    analogWrite(motor1_pwm, 110);    // Full speed for Motor 1
    analogWrite(motor2_pwm, 80);    // Reduced speed for Motor 2
  }
}
void tornadoSearch() {
  moveForward();
  delay(100); // Move forward for a short time
  turnRight();
  delay(35); // Adjust turning delay for consistent circling motion
}
