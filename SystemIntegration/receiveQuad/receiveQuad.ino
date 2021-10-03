#include <Wire.h>

#define SLAVE_ADDRESS 0x04

double desiredAngle;

void setup(){
	Serial.begin(115200);
	Wire.begin(SLAVE_ADDRESS);
	Wire.onReceive(receiveQuad);
	Serial.println("Ready!");
}

void loop(){
	delay(100);
}

void receiveQuad(int byteCount){
  byte currentQuad;
	while(Wire.available()){
		currentQuad = Wire.read();
	}
  Serial.print("Quadrant: ");
	Serial.println(currentQuad);
  desiredAngle = ((double(currentQuad) - 1.0)*(PI/2.0));
  Serial.print("Desired angle: ");
  Serial.print(desiredAngle);
  Serial.println(" radians");
}
