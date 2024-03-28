#include <Adafruit_NeoPixel.h>

//Pin code
const int leftP=6;   
const int leftN=5;  
const int rightP=11; 
const int rightN=10; 

#define R2 2 //Right wheel
#define R1 3 //Left wheel

#define PIN 7
#define NUMPIXELS 4

#define LEFT_BACK_LED 0
#define LEFT_FRONT_LED 3

#define RIGHT_BACK_LED 1
#define RIGHT_FRONT_LED 2

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

const int sensorPins[]={A7,A6,A5,A4,A3,A2,A1,A0};

int sensors[] = {0,0,0,0,0,0,0,0};

const int servoPin = 12; // gripper

//Variables
const int pulseWidthOpen = 1000; //gripper open
const int pulseWidthPartialOpen = 1500; //gripper closed
const int period = 20000; 

int motorSpeedForward = 255; 
int motorSpeedTurn = 255;

int lineCount = 0;  
bool shouldPerformLineCountingAndGrabbing = true;  

volatile long countL = 0;
volatile long countR = 0;

unsigned long lastSpeedAdjustTime = 0;
const unsigned long speedAdjustInterval = 5000;

int turnL = 0;

void ISR_L(){
  countL++;
}

void ISR_R(){
  countR++;
}
void releaseObject();
int calculateLineThreshold();

void setup() {
  strip.begin();
  strip.show();
  
  pinMode(leftP, OUTPUT);
  pinMode(leftN, OUTPUT);
  pinMode(rightP, OUTPUT);
  pinMode(rightN, OUTPUT);

  for(int i = 0; i < 8; i++ ){
    pinMode(sensorPins[i], INPUT); 
  }

  pinMode(R1, INPUT_PULLUP);
  pinMode(R2, INPUT_PULLUP);

  pinMode(servoPin, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(R2), ISR_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R1), ISR_L, CHANGE);
  

  Serial.begin(9600);
  while (!Serial); 
  releaseObject();  // gripper open
  calculateLineThreshold(); //Calculating white line with sensors
}


void loop() {
  getSensorsValues();
  calculateLineThreshold();
  
  // Check if line counting and grabbing should be performed
  if (shouldPerformLineCountingAndGrabbing) {
    performLineCountingAndGrabbing();
  } else {
    // Check if all sensors detect black (indicating the presence of the black box)
    if (sensors[0] && sensors[1] && sensors[2] && sensors[3] && sensors[4] && sensors[5] && sensors[6] && sensors[7]) {
       moveForward();
       if (sensors[0] && sensors[1] && sensors[2] && sensors[3] && sensors[4] && sensors[5] && sensors[6] && sensors[7]) {
         
      // Call the function to drop the object and stop
      dropObjectAndStop();
       } 
       else {
      // If the black box is not detected, continue solving the maze
      solveMaze();
    } 
   }   
 }
}



//Movements

void turnUntilMeetTheLine() {
  int sensorValue0, sensorValue1, sensorValue2, sensorValue3, sensorValueA7, sensorValueA5;

  // Start turning
  analogWrite(rightP, 0);
  analogWrite(rightN, motorSpeedTurn); 
  analogWrite(leftP, motorSpeedTurn);
  analogWrite(leftN, 0);

  // Continuously check sensor values
  do {
    sensorValue0 = analogRead(sensorPins[0]); // Middle sensor 1
    sensorValue1 = analogRead(sensorPins[1]); // Middle sensor 2 (left of middle)
    sensorValue2 = analogRead(sensorPins[2]); // Middle sensor 3 (right of middle)
    sensorValue3 = analogRead(sensorPins[3]); // Middle sensor 4
    sensorValueA7 = analogRead(sensorPins[6]); // Leftmost sensor
    sensorValueA5 = analogRead(sensorPins[5]); // Rightmost sensor

    delay(10);

  } while (!((sensorValue0 > calculateLineThreshold() || sensorValue1 > calculateLineThreshold() || sensorValue2 > calculateLineThreshold() || sensorValue3 > calculateLineThreshold()) &&
             sensorValueA7 < calculateLineThreshold() && sensorValueA5 < calculateLineThreshold()));

  stop();
}

void turnRight(){
  colorTurnRight();
  countL=0;
  countR=0;
  stop();
  while(countL < 20 && countR < 20){
    analogWrite(leftP, 0);
    analogWrite(leftN, 255);
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
  }
  stop();
}


void moveForwardBeforeTurn(){
  colorForward();
  countL=0;
  countR=0;
  while(countL < 12 && countR < 12){
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
  }
}

void moveForward() { //Function to move forward
  colorForward();
  analogWrite(leftP, motorSpeedForward);
  analogWrite(leftN, 0);
  analogWrite(rightP, motorSpeedForward);
  analogWrite(rightN, 0);
}

void stop(){
    analogWrite(leftP, 0);
    analogWrite(leftN, 0);
    analogWrite(rightN, 0);
    analogWrite(rightP, 0);
    delay(500);
    }

//Function to calculate the Threshold value

int calculateLineThreshold() { //function to calculate a value of white color, so robot can be used
   static int calculatedLineThreshold = -1;// in any environment, not depending on strict variable
   if (calculatedLineThreshold != -1) {
      return calculatedLineThreshold;
      }
    int highestValue = 0;
    for (int i = 0; i < 8; i++) {
    int sensorValue = analogRead(sensorPins[i]);
    highestValue = max(highestValue, sensorValue);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensorValue);
  }
  
  calculatedLineThreshold = highestValue + 100;
  Serial.print("Calculated lineThreshold: ");
  Serial.println(calculatedLineThreshold);
  return calculatedLineThreshold;
}

void getSensorsValues(){
  for(int i = 0; i < sizeof(sensorPins) / sizeof(sensorPins[0]);i++ ){
    if(analogRead(sensorPins[i]) > calculateLineThreshold()) {
      sensors[i]=1;
      }
    else{
      sensors[i]=0;
      }
    }
  }


//Functions for the LED
void colorTurnLeft()
{
  strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 0, 255)); // Blue for left side
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 0, 255)); // Blue for left side

  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 255, 0)); // Green for right side
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 255, 0)); // Green for right side

  strip.show();
}

void colorTurnRight()
{
  strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 255, 0)); // Green for left side
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 255, 0)); // Green for left side

  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 0, 255)); // Blue for right side
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 0, 255)); // Blue for right side

  strip.show();
} 

//All red
void colorStop()
{
   for(int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0,255,0));
  }
  strip.show();
}

//All green
void colorForward()
{
   for(int i = 0; i < strip.numPixels(); i++) {
   strip.setPixelColor(i, strip.Color(255,0,0));
  }
  strip.show();
}

  
//Function to solve the maze 

void solveMaze(){
    getSensorsValues();
    
    if(sensors[7]){
      moveForwardBeforeTurn();
      turnRight();
      }
      
    else if(!sensors[0] && !sensors[1] && !sensors[2] && !sensors[3] && !sensors[4]&& !sensors[5] && !sensors[6] && !sensors[7]){
      if (turnL==0){
        moveForwardBeforeTurn();
        }     
      turnUntilMeetTheLine();
      turnL++;
      }
    else{
      turnL = 0;
    if(sensors[6]){
    colorForward();
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 140);
    analogWrite(leftN, 0);
    }
    else if(sensors[1]){
    colorTurnRight();
    analogWrite(rightP, 140);
    analogWrite(rightN, 0);
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    }
    else if(sensors[5]){
    colorForward();
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 230);
    analogWrite(leftN, 0);
    }
    else if(sensors[2]){
    colorTurnLeft();
    analogWrite(rightP, 230);
    analogWrite(rightN, 0);
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    }
    else if(sensors[4] && !sensors[3]){
    colorTurnRight();
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 240);
    analogWrite(leftN, 0);
    }
    else if(sensors[3] && !sensors[4]){
    colorTurnLeft();
    analogWrite(rightP, 240);
    analogWrite(rightN, 0);
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    }
    else{
    colorForward();
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    }
   }
 }
//Function beginning and grab the object

void grabObject() { //gripper drop an object
  controlGripper(pulseWidthOpen);
  delay(1000);
}

void releaseObject() { //gripper - grabs an object
  controlGripper(pulseWidthPartialOpen);
  delay(1000);
}

 void measureAndAdjustSpeed() { //Calculate calibrate speed of both wheels
    long speedLeft = countL / speedAdjustInterval;
    long speedRight = countR / speedAdjustInterval;
    countL = 0; // Reset for next measurement
    countR = 0;
  
    // Adjust motorSpeedForward based on the difference in speed
    if (speedLeft > speedRight) {
      motorSpeedForward -= (speedLeft - speedRight); // Slow down left motor
    } else if (speedRight > speedLeft) {
      motorSpeedForward -= (speedRight - speedLeft); // Slow down right motor
    }
    motorSpeedForward = constrain(motorSpeedForward, 200, 255);
}

void performLineCountingAndGrabbing() { //Function for grabbing and starting
  if (lineCount < 4) {
    moveForward();
    int lineDetected = checkLines();
    if (lineDetected) {
      lineCount++;
      delay(200); 
    }
  }
   if (lineCount == 4 ) {
    stop();
    grabObject();
    delay(500); // Delay to showcase the grab action
    turnUntilMeetTheLine();// Turn left 90 degrees after grabbing the object
    turnL++;
    shouldPerformLineCountingAndGrabbing = false;
    lineCount++; // Increment to prevent re-entering this block
  }
}

int checkLines() { //function to detect black lines in the begining
  for (int i = 0; i < 8; i++) {
    if (analogRead(sensorPins[i]) > calculateLineThreshold()) {
      return 1; // Line detected
    }
  }
  return 0; // No line detected
}

void controlGripper(int pulseWidth) { //Function to control gripper without Servo
  for (int i = 0; i < 50; i++) { // Generate signal for ~1 second
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth); // Send high pulse
    digitalWrite(servoPin, LOW);
    delayMicroseconds(period - pulseWidth); // Complete the period to 20ms
  }
}

//Function to drop the object and finish the maze

void dropObjectAndStop() {
    // Move forward to check if the box is still black
    moveForward();
    
    // Check if all sensors still detect black
    if (sensors[0] && sensors[1] && sensors[2] && sensors[3] && sensors[4] && sensors[5] && sensors[6] && sensors[7]) {
      // Step back a little bit
      analogWrite(leftP, 100);
      analogWrite(leftN, 255);
      analogWrite(rightP, 100);
      analogWrite(rightN, 255);
      delay(500); // Adjust the delay based on how much you want the robot to step back
      
      // Drop the object (open the gripper)
      releaseObject();
      delay(1000); // Delay to showcase the drop action
      
      // Stop the robot
      stop();
    }
  }
