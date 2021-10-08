#include<Wire.h>

#define SLAVE_ADDR 0x04

#define M1_DIR 7
#define M1_PWM 9
#define MOTOR_EN 4

#define RESET_BTN 2
#define ENC_A 3
#define ENC_B 5

float I = 0.0;

float desiredAngle = 0.0;
int encCount = 0;
float encPosition = 0.0;

long startTime, endTime;
long period = 10000;

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

  endTime = micros();

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RESET_BTN), resetISR, HIGH);

  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveQuad);
}

void loop() {
  startTime = micros();
   
  if(firstRun == true){
    delay(1000);
    firstRun = false;
    startTime = micros();
    analogWrite(M1_PWM, 255);
  }
  Serial.println(encPosition);
  while(micros() < startTime + period){
    
  }
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
    desiredAngle = ((double(i) - 1.0)*(PI/2.0));
  }
}
