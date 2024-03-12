#define neckServo 13
#define neckRight 400
#define neckCenter 1300
#define neckLeft 2300


#define trigPin 7
#define echoPin 12

#define leftMotorPin1  5
#define leftMotorPin2  6
#define rightMotorPin1  9
#define rightMotorPin2  10

#define rotaryRight 3
#define rotaryLeft 2



void servoTurn(int angle) {
    digitalWrite(neckServo, HIGH);
    delayMicroseconds(angle);
    digitalWrite(neckServo, LOW);
}

int checkDistance() {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(trigPin, HIGH);
    return duration * 0.034 / 2;
}

void setup() {
    pinMode(neckServo, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);
    pinMode(rotaryRight, INPUT);
    pinMode(rotaryLeft, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}
