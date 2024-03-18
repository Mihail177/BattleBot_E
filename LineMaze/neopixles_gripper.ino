//code added for Neopixels.
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        7 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 4 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

unsigned long greenLightStartTime = 0; // Stores the time the green light was turned on
bool greenLightOn = false;

const int motorA1 = 11;
const int motorA2 = 10;
const int motorB1 = 6;
const int motorB2 = 5;

const int servoPin = 12;


const int pulseWidthOpen = 1000;
const int pulseWidthPartialOpen = 1500;
const int period = 20000;

const int lineSensors[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int motorSpeedForward = 255;
int motorSpeedTurn = 255;

int lineCount = 0;
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

  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  
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

  Serial.begin(38400);
  while (!Serial); 
  releaseObject();
  calculateLineThreshold();
}


void loop() {
  calculateLineThreshold();
  if (shouldPerformLineCountingAndGrabbing) {
    performLineCountingAndGrabbing();
  }
  else {    
    if (millis() - lastSpeedAdjustTime > speedAdjustInterval) {
      measureAndAdjustSpeed();
      lastSpeedAdjustTime = millis();
    }

     updateGreenLight(); //code added for Neopixels.

    int sensorValue0 = analogRead(lineSensors[0]); // Green, middle one
    int sensorValue1 = analogRead(lineSensors[1]); // A1, left of middle
    int sensorValue2 = analogRead(lineSensors[2]); // Blue, middle one
    int sensorValue3 = analogRead(lineSensors[3]); // A3, right of middle
    int sensorValueA7 = analogRead(lineSensors[6]); // A7, leftmost brown (not used for turn logic here)
    int sensorValueA5 = analogRead(lineSensors[5]); // A5, rightmost

    if ((sensorValue0 > calculateLineThreshold() || sensorValue1 > calculateLineThreshold() || sensorValue2 > calculateLineThreshold() || sensorValue3 > calculateLineThreshold())) {
    moveForward();
    } else {
      if (sensorValue1 < sensorValue3) {
          turnRight();
      } else {
        turnLeft();
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

void grabObject() {
  controlGripper(pulseWidthOpen);
  delay(1000);
}

//code added for Neopixels.
void updateGreenLight() {
  // Check if the green light is on and if 1 second has passed
  if (greenLightOn && millis() - greenLightStartTime >= 1000) {
    // Turn off green lights
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.show();
    greenLightOn = false; // Indicate that the green light is now off
  }
}

void releaseObject() {
  controlGripper(pulseWidthPartialOpen);
  delay(1000);
}

void dropObjectIfAllSensorsDetectBlack() {
  unsigned long startTime = millis(); // Start the timer
  bool allBlackDetected = false;

  while (millis() - startTime < 2000) { // Check for 1 second
    allBlackDetected = true; // Assume all are detecting black
    for (int i = 0; i < 8; i++) {
      if (analogRead(lineSensors[i]) < calculateLineThreshold()) {
        allBlackDetected = false; // If any sensor does not detect black, break the loop
        break;
      }
    }

    if (!allBlackDetected) {
      // Reset the timer if the condition was not met and start checking again
      startTime = millis();
    }
    // Insert a small delay to prevent the loop from running too fast
    delay(10);
  }

  if (allBlackDetected) {
    releaseObject(); // Activate the mechanism to drop the object
    stopMotors(); // Stop the robot

    while (true) {
      delay(1000); // A delay to prevent the while loop from consuming too much CPU
    }
  }
}



  void measureAndAdjustSpeed() {
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

void performLineCountingAndGrabbing() {
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

int checkLines() {
  for (int i = 0; i < 8; i++) {
    if (analogRead(lineSensors[i]) > calculateLineThreshold()) {
      return 1; // Line detected
    }
  }
  return 0; // No line detected
}

void controlGripper(int pulseWidth) {
  for (int i = 0; i < 50; i++) { // Generate signal for ~1 second
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth); // Send high pulse
    digitalWrite(servoPin, LOW);
    delayMicroseconds(period - pulseWidth); // Complete the period to 20ms
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
  //Code added for Neopixels.
 if (!greenLightOn) {
    pixels.setPixelColor(0, pixels.Color(0, 50, 0));
    pixels.setPixelColor(3, pixels.Color(0, 50, 0));
    pixels.show();
    greenLightStartTime = millis(); // Record the start time
    greenLightOn = true; // Indicate that the green light is on
  }
  analogWrite(motorA1, motorSpeedTurn);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); // Optionally reduce speed instead of stopping for a smoother turn
  digitalWrite(motorB2, LOW);
}

void turnRight() {
  //Code added for Neopixels.
  if (!greenLightOn) {
    pixels.setPixelColor(1, pixels.Color(0, 50, 0));
    pixels.setPixelColor(2, pixels.Color(0, 50, 0));
    pixels.show();
    greenLightStartTime = millis(); // Record the start time
    greenLightOn = true; // Indicate that the green light is on
  }
  
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
