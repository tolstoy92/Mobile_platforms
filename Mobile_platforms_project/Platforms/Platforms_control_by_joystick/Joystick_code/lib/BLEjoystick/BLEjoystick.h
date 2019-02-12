#ifndef BLEjoystick_h
#define BLEjoystick_h
#include <Arduino.h>
#include <BLEDevice.h>

class BLEjoystick {

    public:
        void BLEjoystickSetup();
        void GetDataFromJoystick(uint8_t JoystickxPin,uint8_t JoystickyPin);
        //void SetupUUID(std::string ServiceUUID, std::string CharUUID);
        //std::string serviceUUID;
        //std::string charUUID;
    private:
        short xCoordinate;
        short yCoordinate;
        uint8_t solidStatus;

};

#endif