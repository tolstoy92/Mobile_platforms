#ifndef MotorController_h
#define MotorController_h
#include "Arduino.h"//стандартная библиотека  Arduino.h
#include <SPI.h>     //библиотека для работы с шиной SPI

class MotorController //создаем класс RadioJoystick
{

private:

public:
//Задаем подключеные соответствующие контакты, для правого и левого двигателей, в очередности pinUpRight, pinDownRight, pinSpeedRight, pinUpLeft, pinDownLeft, pinSpeedLeft
	void setup();

	void moveForward(short speed);

	void moveBack(short speed);

	void turnRight(short speed);

	void turnLeft(short speed);

	void stopMovement();

	void setupMotorDriver(short frequency, uint8_t resolution);

};
#endif