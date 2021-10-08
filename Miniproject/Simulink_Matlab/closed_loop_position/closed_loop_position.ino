#include<Wire.h>

#define SLAVE_ADDR 0x04

#define M1_DIR 7
#define M1_PWM 9
#define MOTOR_EN 4

#define RESET_BTN 2
#define ENC_A 3
#define ENC_B 5

#define Kp 0.253733882698412
#define K 225
#define Ki 0.0161502759705219

//const float Kp = 0.2274788; 
//const float K = 6.4;
//const float Ki = 0.0517371; 


float I = 0.0;
float error_past = 0.0;

float desiredAngle = 0.0;
int encCount = 0;
float encPosition = 0.0;

long startTime;
long period = 10000;
long Ts = 0;
long Tc;

bool firstRun = true;

void setup() {
  Serial.begin(2000000);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(RESET_BTN, INPUT_PULLUP);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, HIGH);
  digitalWrite(M1_DIR, HIGH);

  Tc = micros();

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RESET_BTN), resetISR, HIGH);

  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveQuad);
}

void loop() {
  startTime = micros();
  PIControl();
  while(micros() < startTime + period){
    
  }

  startTime = micros();
   
  if(firstRun == true){
    delay(1000);
    firstRun = false;
    startTime = micros();
    desiredAngle = PI;
  }
  Serial.println(encPosition);
  PIControl();
  while(micros() < startTime + period){
    
  }
}

void PIControl(){
  float error = desiredAngle - encPosition;
  
  if(error > PI)error -= 2*PI;
  else if(error < -1*PI)error += 2*PI;
  
//  Serial.println(error);
  I = I + (Ts/10000)*error;
  float u = K*(Kp*error + Ki*I*error);
  
  if(u > 0) digitalWrite(M1_DIR, HIGH);
  else digitalWrite(M1_DIR, LOW);
  analogWrite(M1_PWM, abs(u));
  Ts = micros() - Tc;
  Tc = micros();
}

void encoderISR(){
  int encAStatus = digitalRead(ENC_A);
  int encBStatus = digitalRead(ENC_B);
  if(encAStatus == encBStatus){
    encCount += 2;
    if(encCount == 3202) encCount = 2;
  }else{
    encCount -= 2;
    if(encCount == -2) encCount = 3198;
  }
  encPosition = (float(encCount)*2.0*PI)/3200.0;
  //Serial.println(encPosition);
}

void resetISR(){
  desiredAngle = 0.0;
}

void receiveQuad(int byteCount){
  byte currentQuad;
  while(Wire.available()){
    currentQuad = Wire.read();
  }
  desiredAngle = ((double(currentQuad) - 1.0)*(PI/2.0));
}

void serialEvent(){
  while(Serial.available() > 0){
    int i = Serial.read();
    int j = Serial.read();
//    Serial.println(i);
    if(i == 49) desiredAngle = 0.0;
    if(i == 50) desiredAngle = PI/2.0;
    if(i == 51) desiredAngle = PI;
    if(i == 52) desiredAngle = (3*PI)/2.0;
    Serial.println(desiredAngle);
  }
}
