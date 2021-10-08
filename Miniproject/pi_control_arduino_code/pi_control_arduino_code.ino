#include<Wire.h>

#define SLAVE_ADDR 0x04 // Address that the arduino is listening at

#define M1_DIR 7
#define M1_PWM 9
#define MOTOR_EN 4 // Definitions for the pins that correspond to motor control

#define RESET_BTN 2
#define ENC_A 3
#define ENC_B 5 // Definitions for encoder channels and reset buttons

#define Kp 0.253733882698412
#define K 225
#define Ki 0.0161502759705219 // PI control gain values from Simulink simulation

float I = 0.0;
float error_past = 0.0; // Globals to keep track of integral and error for the PI

float desiredAngle = 0.0; // Angle in radians that is sent from raspi
int encCount = 0; // The count of the encoder
float encPosition = 0.0; // The angle in radians that the encoder is at (0-2pi)

long startTime; // Used to keep track of the number of microseconds that the main loop has been running for
long period = 10000; // Set the period for running the PI controller to 10ms (10000us)
long Ts = 0; 
long Tc; // Used to keep track of the number of microseconds since last PI control

void setup() {
  Serial.begin(2000000);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP); // Pullup input so that external resistors are not needed
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT); // Motor 1 direction(voltage) and speed
  pinMode(RESET_BTN, INPUT_PULLUP); // Reset button set to pullup
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, HIGH); // Enable motor control
  digitalWrite(M1_DIR, HIGH); // Set initial direction to CCW

  Tc = micros(); // initialize Tc

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RESET_BTN), resetISR, HIGH); // Interrupt for encoder and reset button

  Wire.begin(SLAVE_ADDR); // Start listening for I2C
  Wire.onReceive(receiveQuad); // Interrupt handler for I2C input
}

void loop() {
  startTime = micros(); // Start timing main loop
  PIControl(); // Run the PI control logic
  while(micros() < startTime + period){ // Wait until 10ms has passed since the start of main loop
    
  }
}

void PIControl(){
  float error = desiredAngle - encPosition; // Set error to difference between angle set by raspi and the current position from the encoder
  
  if(error > PI)error -= 2*PI;
  else if(error < -1*PI)error += 2*PI; // This logic makes sure that the error is compensated even for rollover (since 2pi = 0)
  
  I = I + (Ts/10000)*error; // Take the integral term as the previous integral plus the product of the error term and loop time (converted from us -> s)
  float u = K*(Kp*error + Ki*I*error); // Set the PWM motor output as the product of the P/I gains and error and multiply by the overall gain
  
  //u = constrain(u, -255, 255); // Make sure the output of the PI controller is a valid PWM signal

  if(u > 0) digitalWrite(M1_DIR, HIGH);
  else digitalWrite(M1_DIR, LOW); // If the PI control output is > 0 , turn CCW, otherwise turn CW
  analogWrite(M1_PWM, abs(u)); // Write the PI control output to PWM
  Ts = micros() - Tc;
  Tc = micros(); // Reset timers for next loop
}

void encoderISR(){
  int encAStatus = digitalRead(ENC_A);
  int encBStatus = digitalRead(ENC_B); // Get the current status of the encoder
  
  if(encAStatus == encBStatus){ // Since the ISR is invoked by encoder a changing, we can just check if a = b
    encCount += 2; // Increase count by 2 since the encoder has moved 2 clicks CCW
    if(encCount == 3202) encCount = 2; // If the encoder is at its max, rollover to 0
  }else{ // encAStatus != encBStatus
    encCount -= 2; // Decrease count by 2 since encoder has moved 2 clicks CW
    if(encCount == -2) encCount = 3198; // If encoder is at its minimum, rollover to 3200
  }
  
  encPosition = (float(encCount)*2.0*PI)/3200.0; // Convert encoder count to angle in radians
}

void resetISR(){
  desiredAngle = 0.0; // If reset button is pushed set the desired angle to 0 rads
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
    int i = Serial.read();
    desiredAngle = ((double(i) - 1.0)*(PI/2.0));
  }
}
