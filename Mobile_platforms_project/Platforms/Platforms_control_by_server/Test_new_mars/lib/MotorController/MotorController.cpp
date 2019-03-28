#include "MotorController.h"     //библиотека для управления двигателями
#include "Arduino.h"            //стандартная библиотека  Arduino.h
#include <SPI.h>                //библиотека для работы с шиной SPI
#include "esp32-hal-ledc.h" //библиотека ШИМ
#include <cmath>   

#define PIN_STANDBY 23
#define PIN_ENABLE_R 5
#define PIN_FORWARD_R 19
#define PIN_BACK_R 18
#define PIN_FORWARD_L 4
#define PIN_BACK_L 2
#define PIN_ENABLE_L 15
#define CHANNEL_R 0
#define CHANNEL_L 1


void MotorController::setup()
{
	pinMode(PIN_STANDBY, OUTPUT);
	pinMode(PIN_ENABLE_R, OUTPUT);
	pinMode(PIN_ENABLE_L, OUTPUT);
	pinMode(PIN_FORWARD_R, OUTPUT);
	pinMode(PIN_FORWARD_L, OUTPUT);
	pinMode(PIN_BACK_R, OUTPUT);
	pinMode(PIN_BACK_L, OUTPUT);
	
}

void MotorController::setupMotorDriver(short frequency, uint8_t resolution)
{
	ledcSetup(CHANNEL_R,frequency,resolution);
	ledcSetup(CHANNEL_L,frequency,resolution);
	ledcAttachPin(PIN_ENABLE_R, CHANNEL_R);
	ledcAttachPin(PIN_ENABLE_L, CHANNEL_L);
}

void MotorController::moveForward(short speed)
{
	digitalWrite(PIN_STANDBY, HIGH);
	digitalWrite(PIN_FORWARD_R, LOW);
	digitalWrite(PIN_BACK_R, HIGH);
	digitalWrite(PIN_FORWARD_L, HIGH);
	digitalWrite(PIN_BACK_L, LOW);
	ledcWrite(CHANNEL_R, speed);
	ledcWrite(CHANNEL_L, speed);
}

void MotorController::moveBack(short speed)
{
	digitalWrite(PIN_STANDBY, HIGH);
	digitalWrite(PIN_FORWARD_R, HIGH);
	digitalWrite(PIN_BACK_R, LOW);
	digitalWrite(PIN_FORWARD_L, LOW);
	digitalWrite(PIN_BACK_L, HIGH);
	ledcWrite(CHANNEL_R, speed);
	ledcWrite(CHANNEL_L, speed);
}

void MotorController::turnRight(short speed)
{
	digitalWrite(PIN_STANDBY, HIGH);
	digitalWrite(PIN_FORWARD_R, HIGH);
	digitalWrite(PIN_BACK_R, LOW);
	digitalWrite(PIN_FORWARD_L, HIGH);
	digitalWrite(PIN_BACK_L, LOW);
	ledcWrite(CHANNEL_R, speed);
	ledcWrite(CHANNEL_L, speed);
}

void MotorController::turnLeft(short speed)
{
	digitalWrite(PIN_STANDBY, HIGH);
	digitalWrite(PIN_FORWARD_R, LOW);
	digitalWrite(PIN_BACK_R, HIGH);
	digitalWrite(PIN_FORWARD_L, LOW);
	digitalWrite(PIN_BACK_L, HIGH);
	ledcWrite(CHANNEL_R, speed);
	ledcWrite(CHANNEL_L, speed);
}

void MotorController::stopMovement()
{
	digitalWrite(PIN_STANDBY, HIGH);
	digitalWrite(PIN_FORWARD_R, LOW);
	digitalWrite(PIN_BACK_R, HIGH);
	digitalWrite(PIN_FORWARD_L, HIGH);
	digitalWrite(PIN_BACK_L, LOW);
	ledcWrite(CHANNEL_R, 0);
	ledcWrite(CHANNEL_L, 0);
}



