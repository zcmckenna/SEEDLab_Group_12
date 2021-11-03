#define M1_DIR 7
#define M1_PWM 9
#define M2_DIR 8
#define M2_PWM 10
#define MOTOR_EN 4 // All the motor shield pins

#define ENC_RA 2
#define ENC_RB 5
#define ENC_LA 3
#define ENC_LB 6 // All the channels for the encoders

#define trackWidth 0.283 // The distance between the wheels in m
#define wheelRadius 0.074 // The radius of the wheels in m

#define Period 5000 // The delay time for the main loop in us (5ms)

float angVelR;
float angVelL;

long isrTimeR;
long isrTimeL;

long encoderCountR;
long encoderCountL;
long prevEncoderCountR;
long prevEncoderCountL;

void setup(){
    Serial.begin(2000000);
    pinMode(ENC_RA, INPUT_PULLUP);
    pinMode(ENC_RB, INPUT_PULLUP);
    pinMode(ENC_LA, INPUT_PULLUP);
    pinMode(ENC_LB, INPUT_PULLUP); // Set up all the encoders to use an internal pullup resistor

    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT); // Set up all the motor pins as outputs

    digitalWrite(MOTOR_EN, HIGH); // Enable the motor controller
    digitalWrite(M1_DIR, HIGH);
    digitalWrite(M2_DRI, HIGH); // Set the initial motor directions as CCW

    attachInterrupt(digitalPinToInterrupt(ENC_RA), encoderISR_R, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_LA), encoderISR_L, CHANGE);

    // TODO: initialize I2C stuff
}

void loop(){

}

void encoderISR_R(){
    if(digitalRead(ENC_RA) == digitalRead(ENC_RB))encoderCountR += 2;
    else encoderCountR -= 2;

    angVelR = (float(encoderCountR - prevEncoderCountR)*((2.0*PI)/3200.0)*1000000)/(micros() - isrTimeR);
    isrTimeR = micros();
    prevEncoderCountR = encoderCountR;
}

void encoderISR_L(){
    if(digitalRead(ENC_LA) == digitalRead(ENC_LB))encoderCountL += 2;
    else encoderCountL -= 2;
    
    isrTimeL = micros();
    angVelR = (float(encoderCountL - prevEncoderCountL)*((2.0*PI)/3200.0)*1000000)/(micros() - isrTimeL);
    prevEncoderCountL = encoderCountL;
}