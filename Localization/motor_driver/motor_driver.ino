#include <Wire.h>

#define SLAVE_ADDRESS 0x04

double desiredAngle =0.0;

int inputCLK = 5;                              // CLK input assigned to pin 5
int inputDT = 3;                              // DT input assigned to pin 3
int enablePin = 4;                            // Tri-state enablePin set to pin 4
int M1DIR = 7;                                 //Motor 1 direction set to pin 7                       
int M2DIR = 8;                             
int M1PWM = 9;                                //Motor 1 PWM input set to pin 9
int M2PWM = 10;
int SFlag = 12;                               // Status flag indicator set to pin 12
int resetBtn = 2;                             

int currentStateCLK = LOW;                // variable for current state of the CLK 
int prevStateCLK = currentStateCLK;      // variable for the previous state of CLK
volatile float counter = 0;               // variable that counts the number of rotations. -1 for CCW and +1 for CW
volatile float oldRad = 0;                //variable stores old angular position 
volatile float newRad = 0;                //variable for new angular position calculated from encoder interrupt                     

volatile float timeDiff = 0;



//const long unsigned int minTime = 1;     //minimum amount of time for the ISR to run
long unsigned int lastSwitch;             //stores the time the last ISR ran
int dir;


void setup() {
  Serial.begin (230400);                  //high baud rate used since the encoder has a high resolution                   

  pinMode (inputCLK, INPUT_PULLUP);               
  pinMode (inputDT, INPUT_PULLUP);  
  pinMode (enablePin, OUTPUT);
  pinMode (M1DIR, OUTPUT);
  pinMode (M2DIR, OUTPUT);
  pinMode (M1PWM, OUTPUT);
  pinMode (M2PWM, OUTPUT);
  pinMode (SFlag, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inputDT), ISR_encoder, CHANGE);       //attach the ISR to the DT input to check for changes
  attachInterrupt(digitalPinToInterrupt(resetBtn), ISR_reset, HIGH);          //attach an ISR to the push-button pin that is called when it is HIGH
  
  digitalWrite(enablePin, HIGH);                        //set the enable pin HIGH 
  digitalWrite(SFlag, HIGH);                            //set the status flag HIGH

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveQuad);
}

void loop() {
      if(desiredAngle == 0.0){                        //runs section if the desired angle is 0
        if((newRad<(6.28-0.02)) && (newRad>(0.0+0.02))){   //motor is turned on until the current angular position is within 0.02 radians of the desired angle
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
        analogWrite(M1PWM,0);                        //runs section if the motor's position is at the desired angle and turns off the motor
        digitalWrite(M1DIR, LOW);
        digitalWrite(enablePin,LOW);
      }
      }
      else{
        if(newRad<(desiredAngle-0.02) || newRad>(desiredAngle+0.02)){       //runs section for the other possible angular position set points
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
      
      
}

void ISR_encoder() {
        currentStateCLK = digitalRead(inputCLK);
        if ((prevStateCLK == LOW) && (currentStateCLK == HIGH)) {       //checks if the last CLK state was low and the current CLK state is high
          
          if (digitalRead(inputDT) == HIGH) {      //if the current state of DT is high, then the encoder must have been turned CW
            counter += 1.0;
            if(counter > 800.0){               //resets counter if the number of counts exceeds 1 revolution 
              counter = 0.0;
            }
            newRad = (counter*6.28)/800.0; //dividing by 800 counts per revolution and multiplying by 2*PI to get radians
            Serial.println(newRad);
          }
          else {
            counter -= 1.0;
            if(counter < 0.0){                //resets counter if the number of counts exceeds 1 revolution in the opposite direction
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

void ISR_reset(){                            //when push-button is HIGH, resets the desired angle to the starting position
  desiredAngle = 0.0;  
  Serial.println(desiredAngle);
  
}

void receiveQuad(int byteCount){            //function receives byte over I2C and performs a calculation to output the desired angle associated with the respective quadrant
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


