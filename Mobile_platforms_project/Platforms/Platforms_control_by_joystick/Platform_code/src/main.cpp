#include <Arduino.h>
#include <MotorController.h>
#include <SPI.h>
#include "esp32-hal-ledc.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEcontrol.h>
#include <ctime>

#define PIN_ENABLE_R 25
#define PIN_FORWARD_R 26
#define PIN_BACK_R 27
#define PIN_FORWARD_L 14
#define PIN_BACK_L 12
#define PIN_ENABLE_L 13

std::string SERV_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
std::string CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

std::string valueFromJoystick;
std::string valuex;
std::string valuey;

short x, y, x1, x2;
short frequency = 40;
short xCoord;
short yCoord;
short countL = 0;
short countR = 0;

uint8_t resolution = 10;
uint8_t splitindex;
uint8_t channel_R = 0;
uint8_t channel_L = 1;
uint8_t solidPin = 23;
uint8_t lhallpin = 33;
uint8_t rhallpin = 32;
uint8_t lhall = 0;
uint8_t rhall = 0;

float reduceSpeed = 3;
float reduceSpeedSide = 10;

bool rotateL = true;
bool rotateR = true;

BLEcontrol Esp32;
MotorController DriveCar;
void driveMotor(short, short);
void parseBLEData(std::string valueFromJoystick);

void setup()
{
    Serial.begin(115200);
    pinMode(lhallpin, INPUT);
    pinMode(rhallpin, INPUT);
    DriveCar.setup(PIN_ENABLE_R,PIN_FORWARD_R,PIN_BACK_R,PIN_FORWARD_L,PIN_BACK_L,PIN_ENABLE_L, channel_R, channel_L);
    DriveCar.setupMotorDriver(channel_R, channel_L, frequency, resolution);
    Esp32.initialize(parseBLEData,SERV_UUID,CHAR_UUID);
    pinMode(solidPin, OUTPUT);
}

void parseBLEData(std::string valueFromJoystick)
{
    splitindex = valueFromJoystick.find("/");
    valuex = valueFromJoystick.substr(0,splitindex);
    valuey = valueFromJoystick.substr(splitindex+1);
    xCoord = atoi(valuex.c_str());
    yCoord = atoi(valuey.c_str());
    driveMotor(xCoord, yCoord);
}
void driveMotor(short x1,short x2)
{
    x = map(x1, 0, 4095, -1023, 1023);
    y = map(x2, 0, 4095, -1023, 1023);
    DriveCar.controlByJoystick(x,y,reduceSpeed,reduceSpeedSide);
    Serial.println(x);
    Serial.println(y);
}   

void loop()
{
    
}