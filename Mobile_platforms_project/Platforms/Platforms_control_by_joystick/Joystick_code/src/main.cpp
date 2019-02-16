#include <Arduino.h>
#include <BLEjoystick.h>
#include "BLEDevice.h"

std::string SERV_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
std::string CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
String joystickData = "";

uint8_t JoystickxPin = 33;
uint8_t JoystickyPin = 32;
uint8_t SolidPin = 23;

BLEjoystick Joystick;

void setup() {

	Joystick.BLEjoystickSetup();
	
} 
void loop() {

	joystickData = Joystick.GetDataFromJoystick(JoystickxPin, JoystickyPin);
	Joystick.SendDataFromJoystick(joystickData);

} 