//Pin code
const int leftP=6;   
const int leftN=5;  
const int rightP=11; //A2
const int rightN=10; //A1

const int sensorPins[]={A7,A6,A5,A4,A3,A2,A1,A0};

int sensors[] = {0,0,1,1,1,1,0,0};

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
  pinMode(leftP, OUTPUT);
  pinMode(leftN, OUTPUT);
  pinMode(rightP, OUTPUT);
  pinMode(rightN, OUTPUT);

  for(int i = 0; i < 8; i++ ){
    pinMode(sensorPins[i], INPUT); 
  }

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  pinMode(servoPin, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(2), ISR_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), ISR_L, CHANGE);
  

  Serial.begin(9600);
  while (!Serial); 
  releaseObject();  // gripper open
  calculateLineThreshold(); //Calculating white line with sensors
}


void loop() {
  calculateLineThreshold();
  if (shouldPerformLineCountingAndGrabbing) { //this part of code makes sure to run this only ones and then ignore
    performLineCountingAndGrabbing();
  }
  else{
  solveMaze(); 
  }
}


//Movements


void turnRight(){
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
      turnLeft();
      turnL++;
      }
    else{
      turnL = 0;
    if(sensors[6]){
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 140);
    analogWrite(leftN, 0);
    }
    else if(sensors[1]){
    analogWrite(rightP, 140);
    analogWrite(rightN, 0);
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    }
    else if(sensors[5]){
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 230);
    analogWrite(leftN, 0);
    }
    else if(sensors[2]){
    analogWrite(rightP, 230);
    analogWrite(rightN, 0);
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    }
    else if(sensors[4] && !sensors[3]){
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 240);
    analogWrite(leftN, 0);
    }
    else if(sensors[3] && !sensors[4]){
    analogWrite(rightP, 240);
    analogWrite(rightN, 0);
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    }
    else{
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    }
   }
 }
//Function for the gripper

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
    turnLeft();// Turn left 90 degrees after grabbing the object
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

void turnLeft() {
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
