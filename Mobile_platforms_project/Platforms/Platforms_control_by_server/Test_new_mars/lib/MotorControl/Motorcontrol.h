#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <AX12A.h>
#include "math.h" 

class MotorControl
{
    public:
        short rotateAngle;

        float targetAngle;
        
        float calculateAngle(short currentXcoordinate, short currentYcoordinate, short currentDirectionVectorXcoordinate, short currentDirectionVectorYcoordinate, short targetXcoordinate, short targetYcoordinate);

        void rotateToAngle(short targetAngle, short wheelRaduis, short robotRadius);

        void moveToDistance(short targetAngle, short wheelRaduis, short robotRadius);
    
        void setupMotor();

        void driveMotor();

        void goForward(short motorSpeed);

        void goBackward(short motorSpeed);

        void turnLeft(short motorSpeed);

        void turnRight(short motorSpeed);

        void stopMovement();

        void enableCentralMotor();

        void leftMotorDirection(short leftMotorDirectionValue, short leftMotorSpeed);

        void rightMotorDirection(short rightMotorDirectionValue, short rightMotorSpeed);

    private:

        short currentVectorX;

        short currentVectorY;

        short targetVectorX;

        short targetVectorY;

        short x0, x1, x2, y0, y1, y2;

        float scalprod, mod1, mod2;



};

#endif