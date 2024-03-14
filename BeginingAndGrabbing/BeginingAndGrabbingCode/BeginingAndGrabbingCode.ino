const int motorA1 = 10;
const int motorA2 = 11;
const int motorB1 = 6;
const int motorB2 = 5;

const int servoPin = 9; // gripper


const int pulseWidthOpen = 1000; //gripper open
const int pulseWidthPartialOpen = 1500; //gripper closed
const int period = 20000; // 

const int lineSensors[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; // 8-pin sensor 

int motorSpeedForward = 255; 
int motorSpeedTurn = 255;

int lineCount = 0;  //
bool shouldPerformLineCountingAndGrabbing = true;  


volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;
unsigned long lastSpeedAdjustTime = 0;
const unsigned long speedAdjustInterval = 5000;

void ISR_encoderCountLeft() {
  encoderCountLeft++;
}

void ISR_encoderCountRight() {
  encoderCountRight++;
}

void setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  pinMode(servoPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(3), ISR_encoderCountLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(2), ISR_encoderCountRight, RISING);

  for (int i = 0; i < 8; i++) {
    pinMode(lineSensors[i], INPUT);
  }

  Serial.begin(9600);
  while (!Serial); 
  releaseObject();  // gripper open
  calculateLineThreshold(); //Calculating white line with sensors
}

}

void loop() {
  calculateLineThreshold(); // If you want to use adaptive treshold but not static, this function should be placed everywhere instead of variable
  if (shouldPerformLineCountingAndGrabbing) { //this part of code makes shure to run this only ones and then ignore
    performLineCountingAndGrabbing();
  }
  else { 
    //place here your code
    //Here you have an example of using adaptive threshold  
//    int sensorValue0 = analogRead(lineSensors[0]); // Green, middle one
//    int sensorValue1 = analogRead(lineSensors[1]); // A1, left of middle
//    int sensorValue2 = analogRead(lineSensors[2]); // Blue, middle one
//    int sensorValue3 = analogRead(lineSensors[3]); // A3, right of middle
//    int sensorValueA7 = analogRead(lineSensors[6]); // A7, leftmost brown (not used for turn logic here)
//    int sensorValueA5 = analogRead(lineSensors[5]); // A5, rightmost
//
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
    long speedLeft = encoderCountLeft / speedAdjustInterval;
    long speedRight = encoderCountRight / speedAdjustInterval;
    encoderCountLeft = 0; // Reset for next measurement
    encoderCountRight = 0;
  
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
  digitalWrite(motorA1, motorSpeedForward);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, motorSpeedForward);
  digitalWrite(motorB2, LOW);
}

void stopMotors() { //function to stop
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void turnLeft() { //function to turn left
  analogWrite(motorA1, motorSpeedTurn);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); // Optionally reduce speed instead of stopping for a smoother turn
  digitalWrite(motorB2, LOW);
}

void turnRight() { //function to turn right
  digitalWrite(motorA1, LOW); // Optionally reduce speed instead of stopping for a smoother turn
  digitalWrite(motorA2, LOW);
  analogWrite(motorB1, motorSpeedTurn);
  digitalWrite(motorB2, LOW);
}


void turnLeft90Degrees() {
  int sensorValue0, sensorValue1, sensorValue2, sensorValue3, sensorValueA7, sensorValueA5;

  // Start turning
  digitalWrite(motorA1, LOW);
  analogWrite(motorA2, motorSpeedTurn); 
  analogWrite(motorB1, motorSpeedTurn);
  digitalWrite(motorB2, LOW);

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
