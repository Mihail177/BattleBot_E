#define neckServo 7
#define neckRight 400
#define neckCenter 1300
#define neckLeft 2300


#define trigPinForward 8
#define echoPinForward 12

#define trigPinLeft 4
#define echoPinLeft A5

#define gripper_pin 11

#define leftB  5
#define leftF  6
#define rightF 9
#define rightB 10

#define rotaryRight 3
#define rotaryLeft 2

volatile int ticksLeft = 0; 
volatile int ticksRight = 0; 

int distanceLeft = 0;
int distanceForward = 0;
boolean isDistanceForwardReached = false;
long duration = 0;

void wait(int waitingTime) {
  int time = millis();
  while(millis() < time + waitingTime){
    }
}

void servoTurn(int angle) {
    digitalWrite(neckServo, HIGH);
    delayMicroseconds(angle);
    digitalWrite(neckServo, LOW);
}

int checkDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(10);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2;
}






void tickLeft() {
  ticksLeft++;
}

void tickRight() {
  ticksRight++;
}

void idle()
{
  analogWrite(leftB, 0);
  analogWrite(leftF, 0);
  analogWrite(rightF, 0);
  analogWrite(rightB, 0);
}

void driveForward(int distance) {
  int rightTicksRemaining = (distance - 7) * 31;
  int leftTicksRemaining = (distance - 7) * 35;

  int currentLeftTicks = 0;
  int currentRightTicks = 0;
  
  noInterrupts(); // Enter critical section
  currentLeftTicks = ticksLeft; // Read left ticks
  currentRightTicks = ticksRight; // Read right ticks
  interrupts(); // Exit critical section
  
  
  if (currentLeftTicks >= leftTicksRemaining || currentRightTicks >= rightTicksRemaining) {
    idle();
  } else {
    analogWrite(leftF, 240); 
    analogWrite(rightF, 255);
  }
  
}

void turnLeft()
{
  int currentLeftTicks = 0;
  int currentRightTicks = 0;
  
  noInterrupts(); // Enter critical section
  currentLeftTicks = ticksLeft; // Read left ticks
  currentRightTicks = ticksRight; // Read right ticks
  interrupts(); // Exit critical section
  
  
  if (currentLeftTicks > 32 || currentRightTicks > 32) {
    isDistanceForwardReached = false;
    idle();
  } else {
    analogWrite(leftF, 0); 
    analogWrite(rightF, 255);
  }
}

void turnBack()
{
  int currentLeftTicks = 0;
  int currentRightTicks = 0;
  
  noInterrupts(); // Enter critical section
  currentLeftTicks = ticksLeft; // Read left ticks
  currentRightTicks = ticksRight; // Read right ticks
  interrupts(); // Exit critical section
  
  
  if (currentLeftTicks >= 38 || currentRightTicks >= 32) {
    isDistanceForwardReached = false;
    idle();
  } else {
    analogWrite(leftF, 240); 
    analogWrite(rightB, 255);
  }
}

void setup() {
    Serial.begin(9600);
    pinMode(neckServo, OUTPUT);
    pinMode(trigPinForward, OUTPUT);
    pinMode(echoPinForward, INPUT);
    pinMode(trigPinLeft, OUTPUT);
    pinMode(echoPinLeft, INPUT);
    pinMode(leftB, OUTPUT);
    pinMode(leftF, OUTPUT);
    pinMode(rightF, OUTPUT);
    pinMode(rightB, OUTPUT);
    pinMode(rotaryRight, INPUT);
    pinMode(rotaryLeft, INPUT);

    attachInterrupt(digitalPinToInterrupt(rotaryLeft), tickLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rotaryRight), tickRight, CHANGE);
    servoTurn(neckCenter);
}

void loop() {
  

  distanceForward = checkDistance(trigPinForward, echoPinForward);
  distanceLeft = checkDistance(trigPinLeft, echoPinLeft);

//  Serial.print("Forward:");
//  Serial.println(distanceForward);
//  Serial.print("Left:");
//  Serial.println(distanceLeft);
  

  
  
  if(distanceLeft >= 15)
  {
    turnLeft();
    wait(200);
  }
  if(distanceLeft < 15 && distanceForward >= 15)
  {
    driveForward(distanceForward); 
    wait(200);
  }
  if(distanceLeft < 15 && distanceForward <= 15)
  {
    turnBack();
    wait(200);
  }

  

}
