#ifndef BLEjoystick_h
#define BLEjoystick_h
#include <Arduino.h>
#include <BLEDevice.h>

class BLEjoystick {

    public:
        void BLEjoystickSetup();
        String GetDataFromJoystick(uint8_t JoystickxPin,uint8_t JoystickyPin);
        void SendDataFromJoystick(String joystickData);
       
    private:
        short xCoordinate;
        short yCoordinate;
        uint8_t solidStatus;

};

#endif