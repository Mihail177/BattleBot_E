//code added for Neopixels.
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIN 13 
#define NUMPIXELS 4 

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

unsigned long greenLightStartTime = 0; // Stores the time the green light was turned on
bool greenLightOn = false;

const int trigPin = 8;
const int echoPin = 7;

const int motorA1 = 10;
const int motorA2 = 11;
const int motorB1 = 6;
const int motorB2 = 5;

const int servoPin = 9;

const int pulseWidthOpen = 1000;
const int pulseWidthPartialOpen = 1500;
const int period = 20000;

const int lineSensors[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int motorSpeedForward = 255;
int motorSpeedTurn = 255;

int lineCount = 0;
int executionStage = 0;
bool shouldPerformLineCountingAndGrabbing = true;

volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;
unsigned long lastSpeedAdjustTime = 0;
const unsigned long speedAdjustInterval = 5000;

bool lastLineDetectedOnLeft = true;
bool lineDetected = false; 

unsigned long actionStartTime = 0;
bool isActionInProgress = false; 
bool isAvoidingObstacle = false;

void ISR_encoderCountLeft() {
  encoderCountLeft++;
}

void ISR_encoderCountRight() {
  encoderCountRight++;
}

void moveForward();
void turnLeft();
void turnRight();
void adjustTurn();
void stopMotors();
void grabObject();
void releaseObject();
void controlGripper(int pulseWidth);
int calculateLineThreshold();
void performLineCountingAndGrabbing();
void measureAndAdjustSpeed();
void updateGreenLight();
void turnLeft90Degrees();
void updateLineDetectionSide();
void LineSeek();
bool blackLineDetectedFor2Seconds();
float getDistance();
void performObstacleAvoidance();
void startObstacleAvoidance();

void setup() {
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif

  pixels.begin();
  
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  pinMode(servoPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(3), ISR_encoderCountLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(2), ISR_encoderCountRight, RISING);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(lineSensors[i], INPUT);
  }

  Serial.begin(9600);
  while (!Serial); 
  releaseObject();
  calculateLineThreshold();
}

void loop() {
  static bool startConditionMet = false; 
  if (executionStage == 0 && !startConditionMet) {
    float startDistance = getDistance();
    if (startDistance > 20) {
      startConditionMet = true; 
    } else {
      return; 
    }
  }
  if (startConditionMet) {
    if (executionStage == 0) {
      if (shouldPerformLineCountingAndGrabbing) {
        performLineCountingAndGrabbing();
      } else {
        measureAndAdjustSpeed();
        updateLineDetectionSide();
        LineSeek();
        adjustTurn();
        float distance = getDistance();
        if (distance < 20) {
          stopMotors();
          startObstacleAvoidance();
          performObstacleAvoidance();
        }
        if (blackLineDetectedFor2Seconds()) {
        stopMotors();
        executionStage = 1; 
        }
      }
    }
    else if (executionStage == 1) {
      moveBackward();
      delay(100);
      releaseObject();
      moveBackward();
      delay(500);
      executionStage = 2;
    } else if (executionStage == 2) {
      stopMotors();
    }
  }
}


bool blackLineDetectedFor2Seconds() {
    static unsigned long startTime = 0; 
    const long detectionPeriod = 100; 
    bool allBlackDetected = true;
    
    for (int i = 0; i < 8; i++) {
        if (analogRead(lineSensors[i]) < calculateLineThreshold()) {
            allBlackDetected = false;
            break;
        }
    }

    if (allBlackDetected) {
        if (startTime == 0) {
            startTime = millis();
        } else if (millis() - startTime >= detectionPeriod) {
            stopMotors();
            startTime = 0; 
            executionStage = 1;
            return true; 
        }
    } else {
        startTime = 0;
    }

    return false;
}
void startObstacleAvoidance() {
  isAvoidingObstacle = true;
  actionStartTime = millis(); 
}

void performObstacleAvoidance() {
  unsigned long actionStartTime = millis();
  unsigned long timeSinceStart = 0;
  bool obstacleAvoided = false; // This flag will be true once the obstacle avoidance sequence is complete.

  while (!obstacleAvoided) {
    timeSinceStart = millis() - actionStartTime;

    if (timeSinceStart < 400) {
      turnRight();
    }
    else if (timeSinceStart >= 400 && timeSinceStart < 1100) {
      moveForward();
    }
    else if (timeSinceStart >= 1100 && timeSinceStart < 1700) {
      turnLeft();
    }
    else {
      moveForward();
      for (int i = 0; i <= 3; i++) {
        if (analogRead(lineSensors[i]) > calculateLineThreshold()) {
          stopMotors();
          obstacleAvoided = true;
          break;
        }
      }
      if (obstacleAvoided) {
          break;
      }
    }
  }
}


int calculateLineThreshold() {
  static int calculatedLineThreshold = -1;
  if (calculatedLineThreshold != -1) {
    return calculatedLineThreshold;
  }

  int highestValue = 0;
  for (int i = 0; i < 8; i++) {
    int sensorValue = analogRead(lineSensors[i]);
    highestValue = max(highestValue, sensorValue);
  }
  
  calculatedLineThreshold = highestValue + 100;
  return calculatedLineThreshold;
}

void grabObject() {
  controlGripper(pulseWidthOpen);
  delay(200);
}

void releaseObject() {
  controlGripper(pulseWidthPartialOpen);
  delay(1000);
}

void updateGreenLight() {
  if (greenLightOn && millis() - greenLightStartTime >= 1000) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.show();
    greenLightOn = false;
  }
}

void measureAndAdjustSpeed() {
    long speedLeft = encoderCountLeft / speedAdjustInterval;
    long speedRight = encoderCountRight / speedAdjustInterval;
    encoderCountLeft = 0; // Reset for next measurement
    encoderCountRight = 0;
  
    if (speedLeft > speedRight) {
      motorSpeedForward -= (speedLeft - speedRight); 
    } else if (speedRight > speedLeft) {
      motorSpeedForward -= (speedRight - speedLeft); 
    }
    motorSpeedForward = constrain(motorSpeedForward, 200, 255);
}

float getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH);

    float distanceCm = duration * 0.034 / 2;

    return distanceCm;
}


void performLineCountingAndGrabbing() {
  static unsigned long lastLineTime = 0;
  unsigned long currentTime = millis();
  
  if (lineCount < 4) {
    moveForward();
    int lineDetected = checkLines();
    if (lineDetected && currentTime - lastLineTime >= 200) {
      lineCount++;
      lastLineTime = currentTime; 
    }
  }
  if (lineCount == 4 ) {
    stopMotors();
    grabObject();
    shouldPerformLineCountingAndGrabbing = false;
    do {
    moveForward();
    } while (!areAllSensorsBelowThreshold());
    lineCount++; 
    turnLeft90Degrees(); 
  }
}

int checkLines() {
  for (int i = 0; i < 8; i++) {
    if (analogRead(lineSensors[i]) > calculateLineThreshold()) {
      return 1; 
    }
  }
  return 0; 
}


void controlGripper(int pulseWidth) {
  for (int i = 0; i < 50; i++) { 
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(period - pulseWidth);
  }
}

void moveForward() {
  digitalWrite(motorA1, motorSpeedForward);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, motorSpeedForward);
  digitalWrite(motorB2, LOW);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void turnLeft() {
 if (!greenLightOn) {
    pixels.setPixelColor(0, pixels.Color(0, 50, 0));
    pixels.setPixelColor(3, pixels.Color(0, 50, 0));
    pixels.show();
    greenLightStartTime = millis(); 
    greenLightOn = true; 
  }
  Serial.print("Turned left ");
  analogWrite(motorA1, motorSpeedTurn);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); 
  digitalWrite(motorB2, LOW);
}

void turnRight() {
  if (!greenLightOn) {
    pixels.setPixelColor(1, pixels.Color(0, 50, 0));
    pixels.setPixelColor(2, pixels.Color(0, 50, 0));
    pixels.show();
    greenLightStartTime = millis();
    greenLightOn = true; 
  }
  Serial.print("Turned right ");
  digitalWrite(motorA1, LOW); 
  digitalWrite(motorA2, LOW);
  analogWrite(motorB1, motorSpeedTurn);
  digitalWrite(motorB2, LOW);
}

void moveBackward() {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
}

void adjustTurn() {
    int sensorValue0 = analogRead(lineSensors[0]); // Green, middle one
    int sensorValue1 = analogRead(lineSensors[1]); // A1, left of middle
    int sensorValue2 = analogRead(lineSensors[2]); // Blue, middle one
    int sensorValue3 = analogRead(lineSensors[3]); // A3, right of middle

   if (sensorValue0 > calculateLineThreshold() || sensorValue1 > calculateLineThreshold()
       || sensorValue2 > calculateLineThreshold() || sensorValue3 > calculateLineThreshold()){
        moveForward();
    } else {
      if (sensorValue1 < calculateLineThreshold()) {
          turnRight();
      }
      else if (sensorValue1 > calculateLineThreshold()) {
          turnLeft();
      }
  }
}

void updateLineDetectionSide() {
    int sensorValue4 = analogRead(lineSensors[4]);
    int sensorValue5 = analogRead(lineSensors[5]);
    int sensorValue6 = analogRead(lineSensors[6]);
    int sensorValue7 = analogRead(lineSensors[7]);

    if (sensorValue6 > calculateLineThreshold() || sensorValue7 > calculateLineThreshold()) {
        lastLineDetectedOnLeft = true;
    } else if (sensorValue4 > calculateLineThreshold() || sensorValue5 > calculateLineThreshold()) {
        lastLineDetectedOnLeft = false;
    }
}

void LineSeek() {
    if (areAllSensorsBelowThreshold()) {
        do {
            if (lastLineDetectedOnLeft) {
                turnRight();
            } else {
                Serial.println("Turning left due to previous right detection...");
                turnLeft();
            }

            lineDetected = analogRead(lineSensors[0]) > calculateLineThreshold() ||
                           analogRead(lineSensors[1]) > calculateLineThreshold() ||
                           analogRead(lineSensors[2]) > calculateLineThreshold() ||
                           analogRead(lineSensors[3]) > calculateLineThreshold();
            
            delay(10); 
        } while (!lineDetected); 

        stopMotors(); 
    }
}

bool areAllSensorsBelowThreshold() {
    for (int i = 0; i < 8; i++) {
        if (analogRead(lineSensors[i]) > calculateLineThreshold()) {
            return false; 
        }
    }
    return true;
}


void turnLeft90Degrees() {
  int sensorValue0, sensorValue1, sensorValue2, sensorValue3, sensorValueA7, sensorValueA5;
  digitalWrite(motorA1, LOW);
  analogWrite(motorA2, motorSpeedTurn); 
  analogWrite(motorB1, motorSpeedTurn);
  digitalWrite(motorB2, LOW);

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
