const int leftP = 11;
const int leftN = 10;
const int rightP = 6;
const int rightN = 5;

const int servoPin = 12; // gripper


const int pulseWidthOpen = 1000; //gripper open
const int pulseWidthPartialOpen = 1500; //gripper closed
const int period = 20000; // 

const int lineSensors[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; // 8-pin sensor 

int motorSpeedForward = 255; 
int motorSpeedTurn = 255;

int lineCount = 0;  //
bool shouldPerformLineCountingAndGrabbing = true;  


volatile long countL  = 0;
volatile long countR  = 0;
unsigned long lastSpeedAdjustTime = 0;
const unsigned long speedAdjustInterval = 5000;

void ISR_countL () {
  countL++;
}

void ISR_countR () {
  countR++;
}

void releaseObject();
int calculateLineThreshold();

void setup() {
  pinMode(leftP, OUTPUT);
  pinMode(leftN, OUTPUT);
  pinMode(rightP, OUTPUT);
  pinMode(rightN, OUTPUT);

  pinMode(servoPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(3), ISR_countL, RISING);
  attachInterrupt(digitalPinToInterrupt(2), ISR_countR, RISING);

  for (int i = 0; i < 8; i++) {
    pinMode(lineSensors[i], INPUT);
  }

  Serial.begin(9600);
  while (!Serial); 
   releaseObject();  // gripper open
  calculateLineThreshold(); //Calculating white line with sensors
}



void loop() {
  calculateLineThreshold(); // If you want to use adaptive treshold but not static, this function should be placed everywhere instead of variable
  if (shouldPerformLineCountingAndGrabbing) { //this part of code makes sure to run this only ones and then ignore
  //performLineCountingAndGrabbing();
  }
  else {
//  if(!startM){start();}
//  solveMaze();
    solveMaze();
 
}

void turnLeft(){
  countL=0;
  countR=0;
  stop();
  while(countL < 20 && countR < 20){
    digitalWrite(leftP, HIGH);
    digitalWrite(leftN, LOW);
    digitalWrite(rightP, LOW);
    digitalWrite(rightN, HIGH);
  }
  stop();
}

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

void turn(){
  countL=0;
  countR=0;
  while(countL < 12 && countR < 12){
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
  }
}

void start(){
  countL=0;
  countR=0;
  while(countL < 60 || countR < 60){
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
  }

  turn();
  turnLeft();
  turn();
  startM = 1;
}
void stop(){
    digitalWrite(leftP, LOW);
    digitalWrite(leftN, LOW);
    digitalWrite(rightN, LOW);
    digitalWrite(rightP, LOW);
    delay(500);
    }

void lineFollower(){
  getSensorsValues();
  if((sensors[2]||sensors[5])&&sensors[4]&&sensors[3]||(sensors[5]&&sensors[4])){
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
  }
  else if(sensors[7]){
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 0);
    analogWrite(leftN, 0);
    }
    //not working for sure
    else if(sensors[0]){
    analogWrite(rightP, 0);
    analogWrite(rightN, 0);
    analogWrite(leftP, 255);
    analogWrite(leftN, 0);
    }
    else if(sensors[6]){
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 127);
    analogWrite(leftN, 0);
    }
    //not working
    else if(sensors[1]){
    analogWrite(rightP, 127);
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
    analogWrite(rightP, 0);
    analogWrite(rightN, 255);
    analogWrite(leftP, 0);
    analogWrite(leftN, 255);
  }
    lineFollower();
  }

void getSensorsValues(){
  for(int i = 0; i < sizeof(sensorPins)/sizeof(sensorPins[0]);i++){
    if(analogRead(sensorPins[i])>700){
      sensors[i]=1;
      }
    else{
      sensors[i]=0;
      }
      Serial.print(sensors[i]);
      Serial.print(" ");
    }
    Serial.println();
  }

void solveTheMaze(){
  getSensorsValues();
  if(sensors[6]==1 && sensors[7]==1){
    do{turnRightForwardFast();getSensorsValues();}
    while(!((sensors[5]==1 || sensors[2]==1)||(sensors[4]==1 && sensors[3]==1)));
    }
  else if((sensors[5]==1 || sensors[2]==1)||(sensors[4]==1 && sensors[3]==1)){goForward();}
  else{
  do{turnLeftBackFast();getSensorsValues();} while(!((sensors[5]==1 || sensors[2]==1)||(sensors[4]==1 && sensors[3]==1)));}
  solveTheMaze();
  delay(10);
  }

void solveMaze(){
    getSensorsValues();
    
    if(sensors[7]){
      turn();
      turnRight();
      }
      
    else if(!sensors[0]&&!sensors[1]&&!sensors[2]&&!sensors[3]&&!sensors[4]&&!sensors[5]&&!sensors[6]&&!sensors[7]){
      if (turnL==0){
        turn();
        }
      
      turnLeft();
      turnL++;
      }
    else{
      turnL =0;
    if(sensors[6]){
    analogWrite(rightP, 255);
    analogWrite(rightN, 0);
    analogWrite(leftP, 140);
    analogWrite(leftN, 0);
    }
    //not working
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

//Movements 
void goBack(){
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, HIGH);
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, HIGH);
}
void stopMoving(){
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, LOW);
}
void goForward(){
  digitalWrite(leftP, HIGH);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, HIGH);
  digitalWrite(rightN, LOW);
}
void turnLeftForward(){
  digitalWrite(leftP, HIGH);
  digitalWrite(leftN, LOW);
}
void turnRightForward(){
  digitalWrite(rightP, HIGH);
  digitalWrite(rightN, LOW);
}
void turnLeftForwardFast(){
  digitalWrite(leftP, HIGH);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, HIGH);
}
void turnRightForwardFast(){
  digitalWrite(rightP, HIGH);
  digitalWrite(rightN, LOW);
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, HIGH);
}
void turnRightBack(){
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, HIGH);
}
void turnLeftBack(){
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, HIGH);
}
void turnRightBackFast(){
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, HIGH);
  digitalWrite(rightP, HIGH);
  digitalWrite(rightN, LOW);
}
void turnLeftBackFast(){
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, HIGH);
  digitalWrite(leftP, HIGH);
  digitalWrite(leftN, LOW);
}
//    if ((sensorValue0 > calculateLineThreshold() || sensorValue1 > calculateLineThreshold() || sensorValue2 > calculateLineThreshold() || sensorValue3 > calculateLineThreshold())) {
//    moveForward();
//    } else {
//      if (sensorValue1 < sensorValue3) {
//          turnRight();
//      } else {
//        turnLeft();
//      }
//    } 
  }
  
  }

  int calculateLineThreshold() { //function to calculate a value of white color, so robot can be used
    static int calculatedLineThreshold = -1;// in any environment, not depending on strict variable
    if (calculatedLineThreshold != -1) {
      return calculatedLineThreshold;
      }
    int highestValue = 0;
    for (int i = 0; i < 8; i++) {
    int sensorValue = analogRead(lineSensors[i]);
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

void grabObject() { //gripper dropes an object
  controlGripper(pulseWidthOpen);
  delay(1000);
}

void releaseObject() { //gripper - grabs an object
  controlGripper(pulseWidthPartialOpen);
  delay(1000);
}

 void measureAndAdjustSpeed() { //I use thisw function to calibrate spped of both wheels
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
      Serial.print("Line Count: ");
      Serial.println(lineCount);
      delay(200); 
    }
  }

  if (lineCount == 4 ) {
    stopMotors();
    grabObject();
    delay(500); // Delay to showcase the grab action
    turnLeft90Degrees(); // Turn left 90 degrees after grabbing the object
    shouldPerformLineCountingAndGrabbing = false;
    lineCount++; // Increment to prevent re-entering this block
  }
}

int checkLines() { //function to detect black lines in the begining
  for (int i = 0; i < 8; i++) {
    if (analogRead(lineSensors[i]) > calculateLineThreshold()) {
      return 1; // Line detected
    }
  }
  return 0; // No line detected
}

void controlGripper(int pulseWidth) { //function to control gripper without Servo
  for (int i = 0; i < 50; i++) { // Generate signal for ~1 second
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth); // Send high pulse
    digitalWrite(servoPin, LOW);
    delayMicroseconds(period - pulseWidth); // Complete the period to 20ms
  }
}

void moveForward() { //function to0 move forward
  digitalWrite(leftP, motorSpeedForward);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, motorSpeedForward);
  digitalWrite(rightN, LOW);
}

void stopMotors() { //function to stop
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, LOW);
}

void turnLeft() { //function to turn left
  analogWrite(leftP, motorSpeedTurn);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, LOW); // Optionally reduce speed instead of stopping for a smoother turn
  digitalWrite(rightN, LOW);
}

void turnRight() { //function to turn right
  digitalWrite(leftP, LOW); // Optionally reduce speed instead of stopping for a smoother turn
  digitalWrite(leftN, LOW);
  analogWrite(rightP, motorSpeedTurn);
  digitalWrite(rightN, LOW);
}


void turnLeft90Degrees() {
  int sensorValue0, sensorValue1, sensorValue2, sensorValue3, sensorValueA7, sensorValueA5;

  // Start turning
  digitalWrite(leftP, LOW);
  analogWrite(leftN, motorSpeedTurn); 
  analogWrite(rightP, motorSpeedTurn);
  digitalWrite(rightN, LOW);

  // Continuously check sensor values
  do {
    sensorValue0 = analogRead(lineSensors[0]); // Middle sensor 1
    sensorValue1 = analogRead(lineSensors[1]); // Middle sensor 2 (left of middle)
    sensorValue2 = analogRead(lineSensors[2]); // Middle sensor 3 (right of middle)
    sensorValue3 = analogRead(lineSensors[3]); // Middle sensor 4
    sensorValueA7 = analogRead(lineSensors[6]); // Leftmost sensor
    sensorValueA5 = analogRead(lineSensors[5]); // Rightmost sensor

  } while (!((sensorValue0 > calculateLineThreshold() || sensorValue1 > calculateLineThreshold() || sensorValue2 > calculateLineThreshold() || sensorValue3 > calculateLineThreshold()) &&
             sensorValueA7 < calculateLineThreshold() && sensorValueA5 < calculateLineThreshold()));

  stopMotors();
}
