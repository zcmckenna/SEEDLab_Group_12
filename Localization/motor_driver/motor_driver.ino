#include <Wire.h>

#define SLAVE_ADDRESS 0x04

double desiredAngle =0.0;

int inputCLK = 5;                              // CLK input assigned to pin 4
int inputDT = 3;                              // DT input assigned to pin 3
int enablePin = 4;
int M1DIR = 7;
int M2DIR = 8;
int M1PWM = 9;
int M2PWM = 10;
int SFlag = 12;
int resetBtn = 2;

int currentStateCLK = LOW;                // variable for current state of the CLK 
int prevStateCLK = currentStateCLK;      // variable for the previous state of CLK
volatile float counter = 0;               // variable that counts the number of rotations. -1 for CCW and +1 for CW
volatile float oldRad = 0;
volatile float newRad = 0;

volatile float timeDiff = 0;



//const long unsigned int minTime = 1;     //minimum amount of time for the ISR to run
long unsigned int lastSwitch;             //stores the time the last ISR ran
int dir;


void setup() {
  Serial.begin (230400);                    

  pinMode (inputCLK, INPUT_PULLUP);               
  pinMode (inputDT, INPUT_PULLUP);  
  pinMode (enablePin, OUTPUT);
  pinMode (M1DIR, OUTPUT);
  pinMode (M2DIR, OUTPUT);
  pinMode (M1PWM, OUTPUT);
  pinMode (M2PWM, OUTPUT);
  pinMode (SFlag, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inputDT), ISR_encoder, CHANGE);       //attach the ISR to the DT input to check for changes
  attachInterrupt(digitalPinToInterrupt(resetBtn), ISR_reset, HIGH);
  
  digitalWrite(enablePin, HIGH);
  digitalWrite(SFlag, HIGH);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveQuad);
}

void loop() {
      if(desiredAngle == 0.0){
        if((newRad<(6.28-0.02)) && (newRad>(0.0+0.02))){
        digitalWrite(enablePin, HIGH);
        analogWrite(M1PWM,50);
        if((newRad > 0) && (newRad <= 3.14)){
          digitalWrite(M1DIR, LOW);
        }
        else if((newRad <= 6.28) && (newRad > 3.14)){
          digitalWrite(M1DIR, HIGH);
        }
      }
      else{
        analogWrite(M1PWM,0);
        digitalWrite(M1DIR, LOW);
        digitalWrite(enablePin,LOW);
      }
      }
      else{
        if(newRad<(desiredAngle-0.02) || newRad>(desiredAngle+0.02)){
        digitalWrite(enablePin, HIGH);
        analogWrite(M1PWM,50);
        if(newRad > desiredAngle){
          digitalWrite(M1DIR, LOW);
        }
        else{
          digitalWrite(M1DIR, HIGH);
        }
      }
      else{
        analogWrite(M1PWM,0);
        digitalWrite(M1DIR, LOW);
        digitalWrite(enablePin,LOW);
      }
      }
     // PIcontroller();
      
      
}

void ISR_encoder() {
        currentStateCLK = digitalRead(inputCLK);
        if ((prevStateCLK == LOW) && (currentStateCLK == HIGH)) {       //checks if the last CLK state was low and the current CLK state is high
          
          if (digitalRead(inputDT) == HIGH) {      //if the current state of DT is high, then the encoder must have been turned CW
            counter += 1.0;
            if(counter > 800.0){
              counter = 0.0;
            }
            newRad = (counter*6.28)/800.0; //dividing by 64 counts per revolution
            Serial.println(newRad);
          }
          else {
            counter -= 1.0;
            if(counter < 0.0){
              counter = 800.0;
            }
            newRad = (counter*6.28)/800.0;
            Serial.println(newRad);
          }
        
          oldRad = newRad;    
        }
      prevStateCLK = currentStateCLK;  
    }
//}

void ISR_reset(){
  desiredAngle = 0.0;  
  Serial.println(desiredAngle);
  
}

void receiveQuad(int byteCount){
  byte currentQuad;
  while(Wire.available()){
    currentQuad = Wire.read();
  }
  //Serial.print("Quadrant: ");
  //Serial.println(currentQuad);
  desiredAngle = ((double(currentQuad) - 1.0)*(PI/2.0));
  //Serial.print("Desired angle: ");
  Serial.print(desiredAngle);
  Serial.println(" radians");
}

/*
void PIcontroller() {
  // Put this code in a loop within the main code, probably only active when the wheel is not independently moving
  // previous to the loop below, you should set desPos to the position that you wish to maintain
  // Make sure that AngularPosition is updated before this point
  newRad = -1*desiredAngle;//whatever you used for the angular position
  //currPos=AngularPosition1;
  
  error=setPosition-;
  
  
  I=I+(Ts/1000)*error;
  
  PWMOutput = error*(Kp+Ki*I);
  

  if(abs(PWMOutput)>255){
    PWMOutput=constrain(PWMOutput,-1,1)*255;
    error = constrain(error,-1,1)*min(255/Kp,abs(error));
  }
 
 
  digitalWrite (VS1, HIGH);
  PWMOutput = abs(PWMOutput);

  analogWrite(outPin, PWMOutput);
  Ts=micros()-Tc;
  Tc=micros();

  
}
*/
