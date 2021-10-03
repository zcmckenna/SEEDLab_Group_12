int inputCLK = 5;                              // CLK input assigned to pin 4
int inputDT = 3;                              // DT input assigned to pin 3
int enablePin = 4;
int M1DIR = 7;
int M2DIR = 8;
int M1PWM = 9;
int M2PWM = 10;
int SFlag = 12;

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
  
  digitalWrite(enablePin, HIGH);
  digitalWrite(SFlag, HIGH);
}

void loop() {
      //analogWrite(M2PWM, 255);

      //pi/2
  
      if(newRad<1.55 || newRad>1.58){
        digitalWrite(enablePin, HIGH);
        analogWrite(M1PWM,50);
        if(newRad > 1.57){
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

void ISR_encoder() {
      //if((millis() - lastSwitch) >= minTime){
        //timeDiff = millis() - lastSwitch;
        //lastSwitch = millis();
        currentStateCLK = digitalRead(inputCLK);
        if ((prevStateCLK == LOW) && (currentStateCLK == HIGH)) {       //checks if the last CLK state was low and the current CLK state is high
          
          if (digitalRead(inputDT) == HIGH) {      //if the current state of DT is high, then the encoder must have been turned CW
            //Serial.print("CW: "); 
            counter += 1.0;
            if(counter > 800.0){
              counter = 0.0;
            }
            newRad = (counter*6.28)/800.0; //dividing by 64 counts per revolution
            Serial.println(newRad);
            //Serial.println(" radians");
            //Serial.println(counter);
          }
          else {
            //Serial.print("CCW: ");
            counter -= 1.0;
            if(counter < 0.0){
              counter = 800.0;
            }
            newRad = (counter*6.28)/800.0;
            Serial.println(newRad);
            //Serial.println(" radians");
            //Serial.println(counter);
          }
        
          oldRad = newRad;    
        }
      prevStateCLK = currentStateCLK;  
    }
//}
