#include <Servo.h>

// Define the pins for the ultrasonic sensor
const int trigPin = 11; // Trigger pin
const int echoPin = 12; // Echo pin
const int servoPin = 13; // Define the pin for the servo motor

const int rotaryRight = 3;
const int rotaryLeft = 2;

volatile int ticksLeft = 0;
volatile int ticksRight = 0;

int ticksLeftNeeded = 0;
int ticksRightNeeded = 0;

// R1 = 2
// R2 = 3

Servo neckServo; // Create a Servo object

const int leftMotorPin1 = 5;  // Pin connected to the motor driver for left motor direction
const int leftMotorPin2= 6;  // Pin connected to the motor driver for left motor direction
const int rightMotorPin1 = 9; // Pin connected to the motor driver for right motor direction
const int rightMotorPin2 = 10; // Pin connected to the motor driver for right motor direction
// Define variables to store the duration of the pulse and the distance
long duration;
int distance;
int distanceForward;
int distanceLeft;

int checkDistance() {
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(trigPin, LOW);

  // Read the duration of the pulse on the echo pin
  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

int checkLeftDistance(){
  neckServo.write(180); // Adjust the angle depending on your setup
//  delay(1000); // Wait for the servo to reach the position
  return checkDistance();
}

int checkRightDistance(){
  neckServo.write(0); // Adjust the angle depending on your setup
  delay(1000); // Wait for the servo to reach the position
  return checkDistance();
  
  
}

void driveRight(){
  analogWrite(leftMotorPin1, 0);
  analogWrite(leftMotorPin2, 237);
  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);
  delay(775);
}

void driveLeft(){
   int currentLeftTicks, currentRightTicks;
  
  noInterrupts(); // Enter critical section
  currentLeftTicks = ticksLeft; // Read left ticks
  interrupts(); // Exit critical section
  
  if (currentLeftTicks > 31) {
    idle();
  } else {
    Serial.print("Left: ");
    Serial.print(currentLeftTicks);
    analogWrite(rightMotorPin1, 255);
  }
}

void driveBack(){
  analogWrite(leftMotorPin1, 200);
  analogWrite(leftMotorPin2, 0);
  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, 100);
  delay(775);
}

//void driveForward() {
//  analogWrite(leftMotorPin1, 0);
//  analogWrite(leftMotorPin2, 240);
//  analogWrite(rightMotorPin1, 255);
//  analogWrite(rightMotorPin2, 0);
//}

void driveForward(int leftTicksRemaining, int rightTicksRemaining) {
  int currentLeftTicks, currentRightTicks;
  distanceLeft = checkLeftDistance();
  
  noInterrupts(); // Enter critical section
  currentLeftTicks = ticksLeft; // Read left ticks
  currentRightTicks = ticksRight; // Read right ticks
  interrupts(); // Exit critical section
  
  
  if (currentLeftTicks > leftTicksRemaining || currentRightTicks > rightTicksRemaining) {
    idle();
  } else {
    Serial.print("Left: ");
    Serial.print(currentLeftTicks);
    Serial.print(", Right: ");
    Serial.println(currentRightTicks);
    analogWrite(leftMotorPin2, 240); 
    analogWrite(rightMotorPin1, 255);
  }
}

void tickLeft() {
  ticksLeft++; 
}

void tickRight() {
  ticksRight++;
}

void idle() {
  analogWrite(leftMotorPin2, 0);
  analogWrite(rightMotorPin1, 0);
}

// Function to stop the robot
void stopRobot() {
  analogWrite(leftMotorPin1, 0);
  analogWrite(leftMotorPin2, 0);
  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);
}



void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  pinMode(rotaryRight, INPUT);
  pinMode(rotaryLeft, INPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(rotaryLeft), tickLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryRight), tickRight, CHANGE);
  // Set the trigger pin as an output and the echo pin as an input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

    // Attach the servo to the pin
  neckServo.attach(servoPin);
  
}

void loop() {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  neckServo.write(80);


  // Calculate the distance in centimeters
  distanceForward = checkDistance();
  ticksLeftNeeded = (distanceForward - 10) * 31;
  ticksRightNeeded = (distanceForward - 10) * 36;
  driveForward(ticksLeftNeeded, ticksRightNeeded);
//driveLeft();

//  if(distanceForward < 12.5)
//  {
//    stopRobot();
//    distanceLeft = checkLeftDistance();
//    distanceRight = checkRightDistance();
//    if(distanceLeft < distanceRight) {
//      driveRight();
//      
//      driveForward();
//    }
//    else if(distanceLeft<25 && distanceRight< 25)
//    {
//      driveBack();
//      
//      driveForward();
//    } else{
//      driveLeft();
//      
//      driveForward();
//    }
//  }
  
  
  // Delay before taking the next measurement
}
