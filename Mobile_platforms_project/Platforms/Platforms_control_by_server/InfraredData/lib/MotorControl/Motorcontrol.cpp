#include "MotorControl.h"
#include "math.h"
#include <Arduino.h>
#include <AX12A.h> 

#define Direction_pin (10u) 
#define Baud_rate (1000000ul) 
#define ID_2 (1u) 
#define ID_1 (2u) 
#define ID_3 (3u)

void MotorControl::enableCentralMotor()
{
    ax12a.torqueStatus(ID_2, ON);
}

void MotorControl::setupMotor() 
{
    ax12a.begin(Baud_rate, Direction_pin, &Serial); 
    ax12a.setEndless(ID_1, ON); 
    ax12a.setEndless(ID_3, ON); 
    ax12a.torqueStatus(ID_2, ON);
}


void MotorControl::rightMotorDirection(short rightMotorDirectionValue, short rightMotorSpeed)
{
    if (rightMotorDirectionValue >= 0) {
        ax12a.turn(ID_3, LEFT, rightMotorSpeed);
    }
    else {
        ax12a.turn(ID_3, RIGHT, rightMotorSpeed);
    }
}

void MotorControl::leftMotorDirection(short leftMotorDirectionValue, short leftMotorSpeed)
{
    if (leftMotorDirectionValue >= 0) {
        ax12a.turn(ID_1, RIGHT, leftMotorSpeed);
    }
    else {
        ax12a.turn(ID_1, LEFT, leftMotorSpeed);
    }
}
void MotorControl::goForward(short motorSpeed)
{
    ax12a.turn(ID_3, LEFT, motorSpeed); 
    ax12a.turn(ID_1, RIGHT, motorSpeed);
}

void MotorControl::goBackward(short motorSpeed)
{
    ax12a.turn(ID_3, RIGHT, motorSpeed); 
    ax12a.turn(ID_1, LEFT, motorSpeed);
}

void MotorControl::turnLeft(short motorSpeed)
{
    ax12a.turn(ID_3, RIGHT, motorSpeed); 
    ax12a.turn(ID_1, RIGHT, motorSpeed);
}

void MotorControl::turnRight(short motorSpeed)
{
    ax12a.turn(ID_3, LEFT, motorSpeed); 
    ax12a.turn(ID_1, LEFT, motorSpeed);
}

void MotorControl::stopMovement()
{
    ax12a.turn(ID_3, RIGHT, 0); 
    ax12a.turn(ID_1, RIGHT, 0);
}

void MotorControl::rotateToAngle(short targetAngle, short wheelRaduis, short robotRadius)
{
    rotateAngle = (360*robotRadius*targetAngle)/(2*wheelRaduis*PI);
    rotateAngle = map(rotateAngle, 0, 300, 0, 1024);
    if (targetAngle>0)
    {
        ax12a.move(ID_3, rotateAngle); 
        ax12a.move(ID_1, -rotateAngle);
    }
    else if (targetAngle<0)
    {
        ax12a.move(ID_3, -rotateAngle); 
        ax12a.move(ID_1, rotateAngle);
    }
}

void MotorControl::moveToDistance(short targetAngle)
{
}

float MotorControl::calculateAngle(short currentXcoordinate, short currentYcoordinate, short currentDirectionVectorXcoordinate, 
short currentDirectionVectorYcoordinate, short targetXcoordinate, short targetYcoordinate)
{
    currentVectorX = currentDirectionVectorXcoordinate-currentXcoordinate;
    currentVectorY = currentDirectionVectorYcoordinate-currentYcoordinate;
    targetVectorX = targetXcoordinate-currentXcoordinate;
    targetVectorY = targetYcoordinate-currentYcoordinate;
    scalprod = (currentVectorX*targetVectorX + currentVectorY*targetVectorY);
    mod1 = sqrt(pow(currentVectorX,2)+ pow(currentVectorY,2));
    mod2 = sqrt(pow(targetVectorX,2)+ pow(targetVectorY,2));
    targetAngle = acos(scalprod/(mod1*mod2));
    if ((targetXcoordinate-currentXcoordinate)*(currentDirectionVectorYcoordinate-currentYcoordinate)*(currentDirectionVectorXcoordinate-currentXcoordinate)>0)
        return targetAngle;
    else if ((targetXcoordinate-currentXcoordinate)*(currentDirectionVectorYcoordinate-currentYcoordinate)*(currentDirectionVectorXcoordinate-currentXcoordinate)<0)
        return -targetAngle;
    else
        return 0;
}
