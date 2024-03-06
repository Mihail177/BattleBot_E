// Motor control pins
const int motorA1 = 10;
const int motorA2 = 11;
const int motorB1 = 6;
const int motorB2 = 5;

// Adjust these PWM values to control motor speed (0-255)
int motorSpeedForward = 200;
int motorSpeedTurn = 200;

// Line sensor pins
const int lineSensors[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Sensor threshold
const int sensorThreshold = 700;

// Variables for speed measurement and adjustment
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;
unsigned long lastSpeedAdjustTime = 0;
const unsigned long speedAdjustInterval = 5000; // Adjust every 5 seconds

// Encoder interrupt service routines (ISR) for left and right wheels
void ISR_encoderCountLeft() {
  encoderCountLeft++;
}

void ISR_encoderCountRight() {
  encoderCountRight++;
}

void setup() {
  // Initialize motor control pins as output
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Initialize encoder input pins and attach interrupts
  attachInterrupt(digitalPinToInterrupt(3), ISR_encoderCountLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(2), ISR_encoderCountRight, RISING);

  Serial.begin(9600);
}

void loop() {
  // Measure speed and adjust if necessary
  if (millis() - lastSpeedAdjustTime > speedAdjustInterval) {
    measureAndAdjustSpeed();
    lastSpeedAdjustTime = millis();
  }

  // Read values from the specified line-following sensors
  int sensorValue0 = analogRead(lineSensors[0]);
  int sensorValue1 = analogRead(lineSensors[1]);
  // Additional sensor readings can be added here

  // Debugging: Print sensor values
  Serial.print("Sensor 0: ");
  Serial.print(sensorValue0);
  Serial.print("\tSensor 1: ");
  Serial.println(sensorValue1);

  // Determine action based on sensor values
  if (sensorValue0 > sensorThreshold || sensorValue1 > sensorThreshold) {
    // If any of the specified sensors detect the line, move forward
    moveForward();
  } else {
    // If none of the sensors detect the line, decide to turn
    if (sensorValue0 < sensorValue1) {
      turnLeft();
    } else {
      turnRight();
    }
  }
  
  delay(100); // Adjust delay as needed
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

  // Ensure motorSpeedForward remains within valid PWM range
  motorSpeedForward = constrain(motorSpeedForward, 0, 255);
}

void moveForward() {
  analogWrite(motorA1, motorSpeedForward);
  digitalWrite(motorA2, LOW);
  analogWrite(motorB1, motorSpeedForward);
  digitalWrite(motorB2, LOW);
}

void turnLeft() {
  digitalWrite(motorA1, LOW); // Optionally reduce speed instead of stopping for a smoother turn
  digitalWrite(motorA2, LOW);
  analogWrite(motorB1, motorSpeedTurn);
  digitalWrite(motorB2, LOW);
}

void turnRight() {
  analogWrite(motorA1, motorSpeedTurn);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); // Optionally reduce speed instead of stopping for a smoother turn
  digitalWrite(motorB2, LOW);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}
