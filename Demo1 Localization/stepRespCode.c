#include<Wire.h>

#define SLAVE_ADDR 0x04 // Address that the arduino is listening at

#define M1_DIR 7
#define M1_PWM 9
#define M2_DIR 8
#define M2_PWM 10
#define MOTOR_EN 4 // Definitions for the pins that correspond to motor control

#define ENC_RA 2 // yellow, pins 3 and 5 for right wheel encoder
#define ENC_RB 5 // white  Definitions for encoder channels and reset buttons
#define ENC_LA 3 //yellow, pins 2 and 6 for left wheel encoder
#define ENC_LB 6 //white

#define rhoK 0.253733882698412
#define phiK 0.253733882698412
#define rhoKp 0.253733882698412
#define phiKp 0.253733882698412
#define K 255
// #define Ki 0.0161502759705219 // PI control gain values from Simulink simulation

#define d 0.283 //distance between wheels in meters
#define r 0.074 //radius of wheel in meters

float I = 0.0;
float error_past = 0.0; // Globals to keep track of integral and error for the PI

float desiredAngle = 0.0; // Angle in radians that is sent from raspi

long startTime; // Used to keep track of the number of microseconds that the main loop has been running for
long period = 10000; // Set the period for running the PI controller to 10ms (10000us)
long Ts = 0; 
long Tc; // Used to keep track of the number of microseconds since last PI control


//variables below measured or calculated inside encoder ISRs **************************************************
int encCountR = 0; // The count of the encoder
int encCountL = 0;
long totalCountR = 0;
long totalCountL = 0;
long totalCountROld = 0;
long totalCountLOld = 0;
float encPositionR = 0.0; // The angle in radians that the encoder is at (0-2pi)
float encPositionL = 0.0;
float encPositionOldR = 0.0;
float encPositionOldL = 0.0;
float timeNewR = 0.0; //variables used to keep track of time to calculate velocity inside encoders
float timeOldR = 0.0;
float timeNewL = 0.0;
float timeOldL = 0.0;

float angVelR = 0.0; //angular velocity of right and left wheels in radians. calculated inside encoder ISRs
float angVelL = 0.0;

float v_r = 0.0; //linear velocity of right and left wheels in meters/seconds. calculated inside encoder ISRs
float v_l = 0.0;
//***********************************************************************************************************

//variables below measured or calculated in main loop *******************************************************
int rolloverR = 0; //counts number of rollovers that have occurred for the right and left wheels
int rolloverL =0;
int rolloverCar = 0; //counts number of rollovers for the rotation of the car 

float rotPosition = 0.0; //robot position calculated in main loop (phi)
float rotVel = 0.0; //robot angular velocity (phi dot)
float avgVel = 0.0; //average velocity of the robot (rho dot)
float xVel = 0.0; //linear velocity of robot with respect to x-axis
float yVel = 0.0; //linear velocity of robot with respect to y-axis

float xPos = 0.0; //x and y position of the robot
float yPos = 0.0; //calculated by multiplying the x and y velocity multiplied by the change in time 

float timeNew =0.0; //variables to keep track of time to calculate the x and y position
float timeOld = 0.0; 
//***********************************************************************************************************

//***********************************************************************************************************
float outerKd; // TODO
float outerKp; // TODO
float outerK; // TODO
float prevDerivativeError;
float turningRate; // the output of the outer control loop (phi dot sub d)
long pdStart = 0;
long pdDelta = 1;

float speedSet = 128; // rhod dot sub d
long pStart = 0;
long pDelta = 1;
//***********************************************************************************************************



void setup() {
  Serial.begin(2000000);
  pinMode(ENC_RA, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP); // Pullup input so that external resistors are not needed
  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP); // Pullup input so that external resistors are not needed
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT); // Motor 1 direction(voltage) and speed
  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); // Motor 1 direction(voltage) and speed
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, HIGH); // Enable motor control
  digitalWrite(M1_DIR, HIGH); 
  digitalWrite(M2_DIR, LOW);// Set initial direction to CCW

  Tc = micros(); // initialize Tc

  attachInterrupt(digitalPinToInterrupt(ENC_RA), encoderISR_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), encoderISR_L, CHANGE);
  
  Wire.begin(SLAVE_ADDR); // Start listening for I2C
  Wire.onReceive(receiveQuad); // Interrupt handler for I2C input
}

void loop() {
    delay(1000);
    analogWrite(M1_PWM, 26);
    analogWrite(M2_PWM, 26);
    long startT = millis();
    long timeKeep;
    while(true){
        timeKeep = micros();
        rotVel = r * (angVelR - angVelL) / d;
        // avgVel = r * (angVelR + angVelL) / 2;

    //code below was used to check position variables. can comment out
        Serial.print(millis());
        Serial.print("\t");  
        Serial.println(rotVel);
        if(millis() > (startT + 2000)){
            break;
        }
        while(micros() < timeKeep + 5000);
    }
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
    while(true);
   
}

void PDControl(){ // Outer control loop (calculating the desired turning rate phi dot sub d)
  float error = desiredAngle - rotPosition;
  float proportional = error;
  float derivative = (error - prevDerivativeError) / pdDelta;
  turningRate = outerKp * proportional + outerKd * derivative;
  turningRate *= outerK;
  prevDerivativeError = error;

  pdDelta = pdStart - micros();
  pdStart = micros();
}

void PControl(){ // Inner control loop (caculating delta V sub a and V bar sub a)
  float phiError = turningRate - rotVel;
  float rhoError = speedSet - avgVel;
  float phiProportional = phiError;
  float rhoProportional = rhoError;

  float deltaVa = phiK * (phiKp * phiProportional);
  float magVa = rhoK * (rhoKp * rhoProportional);

  // TODO calculate PWM values to send to each motor
  float Va1 = (magVa + deltaVa) / 2.0;
  float Va2 = (magVa - deltaVa) / 2.0;
  // Need to constrain values and make sure the correct direction is set (+ vs -)
  analogWrite(M1_PWM, 255 * int(Va1));
  analogWrite(M2_PWM, 255 * int(Va2));
}

void encoderISR_R(){
  int encAStatusR = digitalRead(ENC_RA);
  int encBStatusR = digitalRead(ENC_RB); // Get the current status of the encoder

  timeNewR = micros();
  if(encAStatusR == encBStatusR){ // Since the ISR is invoked by encoder a changing, we can just check if a = b
    encCountR += 2; // Increase count by 2 since the encoder has moved 2 clicks CCW
    totalCountR +=2;
    if(encCountR == 3202){
      encCountR = 2; // If the encoder is at its max, rollover to 0
      rolloverR += 1;
    }
    
  }else{ // encAStatus != encBStatus
    encCountR -= 2; // Decrease count by 2 since encoder has moved 2 clicks CW
    totalCountR -= 2;
    if(encCountR == -2) {
      encCountR = 3198; // If encoder is at its minimum, rollover to 3200
      rolloverR -= 1;
    }
    
  }
  encPositionR = (float(encCountR)*2.0*PI)/3200.0; // Convert encoder count to angle in radians
  angVelR = (float(totalCountR - totalCountROld)*((2.0*PI)/3200.0)*1000000)/(timeNewR - timeOldR);
//   angVelR = ((encPositionR - encPositionOldR)*1000000) / (timeNewR - timeOldR);
  v_r = r * angVelR;
  timeOldR = timeNewR;
//   encPositionOldR = encPositionR;
    totalCountROld = totalCountR;
}

void encoderISR_L(){
  int encAStatusL = digitalRead(ENC_LA);
  int encBStatusL = digitalRead(ENC_LB); // Get the current status of the encoder

  timeNewL = micros();
  if(encAStatusL == encBStatusL){ // Since the ISR is invoked by encoder a changing, we can just check if a = b
    encCountL -= 2; // Decrease count by 2 since encoder has moved 2 clicks CW
    totalCountL -= 2;
    if(encCountL == -2){
      encCountL = 3198; // If encoder is at its minimum, rollover to 3200
      rolloverL -= 1;
    }
  }else{ // encAStatus != encBStatus
    encCountL += 2; // Increase count by 2 since the encoder has moved 2 clicks CCW
    totalCountL += 2;
    if(encCountL == 3202){
      encCountL = 2; // If the encoder is at its max, rollover to 0
      rolloverL += 1;
    }
  }
  angVelL = (float(totalCountL - totalCountLOld)*((2.0*PI)/3200.0)*1000000)/(timeNewL - timeOldL);
  encPositionL = (float(encCountL)*2.0*PI)/3200.0; // Convert encoder count to angle in radians
//   angVelL = ((encPositionL - encPositionOldL)*1000000) / (timeNewL - timeOldL);
//   encPositionL = (float(encCountL)*2.0*PI)/3200.0; // Convert encoder count to angle in radians
//   angVelL = ((encPositionL - encPositionOldL)*1000000) / (timeNewL - timeOldL);
  v_l = r * angVelL;
  timeOldL = timeNewL;
//   encPositionOldL = encPositionL;
totalCountLOld = totalCountL;
}


void receiveQuad(int byteCount){
  byte currentQuad; // Initalize variable to hold new set position from I2C
  while(Wire.available()){ // While the arduino is still getting data over I2C
    currentQuad = Wire.read(); // Set the current quad byte to the byte that was sent over I2C
  }
  desiredAngle = ((double(currentQuad) - 1.0)*(PI/2.0)); // Simple equation to convert from quadrant (1,2,3, or 4) to radians (0pi, pi/2, pi, or 3pi/2)
}

void serialEvent(){
  while(Serial.available() > 0){
    float i = Serial.parseFloat();
    desiredAngle = i + rotPosition;
    // TODO add code to add current rotation to the new desired rotation
  }
}
