const int leftP=6; //11  
const int leftN=5  //10
const int rightP=11;  //6
const int rightN=10; //5
const int sensorPins[]={A7,A6,A5,A4,A3,A2,A1,A0};

int sensors[] = {0,0,1,1,1,1,0,0};

volatile int countL = 0;
volatile int countR = 0;

int turnL = 0;

int startM = 0;

void ISR_L(){
  countL++;
}

void ISR_R(){
  countR++;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(leftP, OUTPUT);
  pinMode(leftN, OUTPUT);
  pinMode(rightP, OUTPUT);
  pinMode(rightN, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), ISR_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), ISR_L, CHANGE);


  Serial.begin(9600);
}

void loop() {
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
