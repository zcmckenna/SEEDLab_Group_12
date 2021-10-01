#include <Wire.h>

#define SLAVE_ADDRESS 0x04

byte currentQuad;

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
	while(Wire.available()){
		currentQuad = Wire.read();
	}
	Serial.println(currentQuad);
}
