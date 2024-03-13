#define neckServo 7
#define neckRight 400
#define neckCenter 1300
#define neckLeft 2300


const int trigPin = 13;
const int echoPin = 12;

#define gripper_pin 11

#define leftB  5
#define leftF  6
#define rightF 9
#define rightB 10

#define rotaryRight 3
#define rotaryLeft 2

volatile int ticksLeft = 0; 
volatile int ticksRight = 0; 

int distanceLeft;
int distanceForward;
boolean isDistanceForwardReached = false;
long duration;



void servoTurn(int angle) {
    digitalWrite(neckServo, HIGH);
    delayMicroseconds(angle);
    digitalWrite(neckServo, LOW);
}

int checkDistance() {
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

void driveForward(int distance) {
  int rightTicksRemaining = (distance - 10) * 31;
  int leftTicksRemaining = (distance - 10) * 36;

  int currentLeftTicks = 0;
  int currentRightTicks = 0;
  
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
    analogWrite(leftF, 240); 
    analogWrite(rightF, 255);
  }
}

void turnLeft()
{
  int currentRightTicks;
  
  noInterrupts(); // Enter critical section
  currentRightTicks = ticksRight; // Read left ticks
  interrupts(); // Exit critical section
  
  if (currentRightTicks > 31) {
    idle();
  } else {
    Serial.print("Right: ");
    Serial.print(currentRightTicks);
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
  
  
  if (currentLeftTicks > 36 || currentRightTicks > 31) {
    isDistanceForwardReached = true;
    idle();
  } else {
    Serial.print("Left: ");
    Serial.print(currentLeftTicks);
    Serial.print(", Right: ");
    Serial.println(currentRightTicks);
    analogWrite(leftB, 240); 
    analogWrite(rightF, 255);
  }
}

void idle()
{
  analogWrite(leftB, 0);
  analogWrite(leftF, 0);
  analogWrite(rightF, 0);
  analogWrite(rightB, 0);
}



void setup() {
    Serial.begin(9600);
    pinMode(neckServo, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(leftB, OUTPUT);
    pinMode(leftF, OUTPUT);
    pinMode(rightF, OUTPUT);
    pinMode(rightB, OUTPUT);
    pinMode(rotaryRight, INPUT);
    pinMode(rotaryLeft, INPUT);

    attachInterrupt(digitalPinToInterrupt(rotaryLeft), tickLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rotaryRight), tickRight, CHANGE);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  servoTurn(neckLeft);

  distanceLeft = checkDistance();

  Serial.println(distanceLeft);

//  if(distanceLeft > 25)
//  {
//    turnLeft();
//    distanceForward = distanceLeft;
//    driveForward(distanceForward);
//  }else if(distanceLeft < 25 && isDistanceForwardReached)
//  {
//    turnBack();
//  }

//driveForward(15);
  
  

}
