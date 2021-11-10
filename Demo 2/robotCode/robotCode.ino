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

#define period 5000 // The delay time for the main loop in us (5ms)

#define sumKp 1
#define sumKi 1
#define diffKp 1
#define diffKi 1

float angVelR;
float angVelL;

long isrTimeR;
long isrTimeL;

long encoderCountR;
long encoderCountL;
long prevEncoderCountR;
long prevEncoderCountL;

float voltageSum; // V bar sub a
float sumIntegral = 0;
float sumOutput;
float voltageDiff; // Delta V sub a
float diffIntegral = 0;
float diffOutput;

float forwardVelocitySet;
float forwardVelocity;
float rotationalVelocity;
float rotationalVelocitySet;
float rotationalPosition;


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
    digitalWrite(M2_DIR, HIGH); // Set the initial motor directions as CCW

    attachInterrupt(digitalPinToInterrupt(ENC_RA), encoderISR_R, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_LA), encoderISR_L, CHANGE);

    // TODO: initialize I2C stuff
}

void loop(){
    long startTime = micros(); // Get the micros at the start of the main loop
    rotationalPosition = wheelRadius * (float(encoderCountR)*((2.0*PI)/3200.0) - float(encoderCountL)*((2.0*PI)/3200.0)) / trackWidth;
    forwardVelocity = wheelRadius * ((angVelR + angVelL) / 2.0);
    rotationalVelocity = wheelRadius * ((angVelR - angVelL) / trackWidth);
    sumPIControl();
    diffPIControl();
    float voltageR = (sumOutput + diffOutput) / 2;
    float voltageL = (sumOutput - diffOutput) / 2;
    if(voltageR >= 0) digitalWrite(M1_DIR, HIGH);
    else digitalWrite(M1_DIR, LOW);
    if(voltageL >= 0) digitalWrite(M2_DIR, HIGH);
    else digitalWrite(M2_DIR, HIGH);
    analogWrite(M1_PWM, constrain(abs(voltageR), 0, 255));
    analogWrite(M2_PWM, constrain(abs(voltageL), 0, 255));
    while(micros() < startTime + period); // Delay for the rest of the period
}

void sumPIControl(){
    float error = forwardVelocitySet -  forwardVelocity;
    float proportional = error;
    sumIntegral = sumIntegral + error * (period / 1000000.0);
    sumOutput = sumKp * proportional + sumKi * sumIntegral;

}

void diffPIControl(){
    float error = rotationalVelocitySet -  rotationalVelocity;
    float proportional = error;
    diffIntegral = diffIntegral + error * (period / 1000000.0);
    diffOutput = diffKp * proportional + diffKi * diffIntegral;
}

void encoderISR_R(){
    if(digitalRead(ENC_RA) == digitalRead(ENC_RB))encoderCountR += 2;
    else encoderCountR -= 2;

    angVelR = (float(encoderCountR - prevEncoderCountR)*((2.0*PI)/3200.0)*1000000)/(micros() - isrTimeR);
    isrTimeR = micros();
    prevEncoderCountR = encoderCountR;
}

void encoderISR_L(){
    if(digitalRead(ENC_LA) == digitalRead(ENC_LB))encoderCountL -= 2;
    else encoderCountL += 2;
    
    angVelL = (float(encoderCountL - prevEncoderCountL)*((2.0*PI)/3200.0)*1000000)/(micros() - isrTimeL);
    isrTimeL = micros();
    prevEncoderCountL = encoderCountL;
}
