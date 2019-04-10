#include <Arduino.h>
#include <Wire.h>
#include <AX12A.h> 
#include "math.h"
#include <I2Cdev.h>
#include <MotorControl.h>
#include <MqttClient.h>
#include <PubSubClient.h>
#include <MPU9250_as.h>
#include <stdio.h>
#include <HardwareSerial.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

std::string receivedData;
std::string sign;
std::string angle;
std::string move;
std::string rotate;
std::string finish;

short constY = -1000;

int mZcount = 0;

short speed = 100;
short correctValue = 0;

uint8_t finishValue = -1;
uint8_t splitindex;
uint8_t rotateValue = -1;
uint8_t moveForwardValue = -1;
uint8_t platformNumber = 2;
uint8_t sensorId;

float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY;
float mZsum = 0;
float mZ = 0;
float targetZ = 0;
float errorY = 0;

const char* ssid = "SPEECH_405";
const char* password = "multimodal";
const char* mqtt_server = "192.168.0.105";

// const char* ssid = "iGarage";
// const char* password = "igarage18";
// const char* mqtt_server = "10.1.30.45";

mqttClient mqtt(ssid, password, mqtt_server);
MotorControl GyroRobot;
MPU9250 mySensor;

void callback(char* topic, byte* message, unsigned int length)
{
    char platformControlTopic[64];

    sprintf(platformControlTopic, "platforms/%d", platformNumber);
    
    if (strcmp(topic, platformControlTopic)==0) {

        receivedData = "";
        sign = "";
        angle = "";
        move = "";
        rotate = "";
        finish = "";
       
        int digit_sign;
        
        for (int i = 0; i < length; i++)
            {
                receivedData += (char)message[i]; 
            } 
            sign = receivedData[0];
            angle = receivedData.substr(1, 3);
            move = receivedData[4];
            rotate = receivedData[5];
            finish = receivedData[6];

            if (sign == "0") {
                digit_sign = -1;
            }
            else {
                digit_sign = 1;
            }

            correctValue = digit_sign * atoi(angle.c_str());
            moveForwardValue = atoi(move.c_str());
            rotateValue = atoi(rotate.c_str());
            finishValue = atoi(finish.c_str());
        } 
} 

void setup()
{
    //Serial.begin(115200);
    #ifdef _ESP32_HAL_I2C_H_ 
    Wire.begin(SDA_PIN, SCL_PIN); 
    #else
    Wire.begin();
    #endif
    GyroRobot = MotorControl();
    GyroRobot.setupMotor();
    // mqtt.setupWifi();
    // mqtt.setCallback(*callback);
    // mqtt.subscribe(platformNumber);
    mySensor.setWire(&Wire);
    mySensor.beginAccel();
    mySensor.beginGyro();
    mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
    mySensor.magZOffset = +207;

    sensorId = mySensor.readId();

}

void loop()
{
    GyroRobot.enableCentralMotor();
    // mqtt.initClientLoop();
    // if (moveForwardValue == 1 && rotateValue == 0) {
    //     GyroRobot.goForward(speed);
    // }
    // if (rotateValue == 0 && moveForwardValue == 0) {
    //     GyroRobot.stopMovement();
    // }
    // if (rotateValue == 1 && moveForwardValue == 0)
    // {
    //    GyroRobot.stopMovement();
    //     if (correctValue > 0) {
    //     GyroRobot.turnLeft(speed);
    //     }
    //     else if (correctValue < 0) {
    //     GyroRobot.turnRight(speed);
    //     }
    // }   
    // else if (finishValue == 1) {
    // GyroRobot.stopMovement();
    // }
    //mqtt.pubFeedback(outputData,platformNumber);

    //Serial.println("sensorId: " + String(sensorId));

    mySensor.accelUpdate();
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    Serial.println("accelX: " + String(aX));
    Serial.println("accelY: " + String(aY));
    Serial.println("accelZ: " + String(aZ));
    Serial.println("accelSqrt: " + String(aSqrt));


    mySensor.gyroUpdate();
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    Serial.println("gyroX: " + String(gX));
    Serial.println("gyroY: " + String(gY));
    Serial.println("gyroZ: " + String(gZ));
  
    mySensor.magUpdate();
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();

    mDirection = mySensor.magHorizDirection();
    Serial.println("magX: " + String(mX));
    Serial.println("maxY: " + String(mY));
    Serial.println("magZ: " + String(mZ));
    Serial.println("mZsum: " + String(targetZ));
    Serial.println("horizontal direction: " + String(mDirection));
  
    Serial.println("at " + String(millis()) + "ms");
    Serial.println(""); // Add an empty line 
    if (constY == -1000) {
        constY = mY;
    }
    Serial.println(constY);
    if ((mY > (constY - 10)) && (mY < (constY + 10))) {
        GyroRobot.goForward(speed*0.7);
        Serial.println("Move");
    }
    else if (mY <= (constY - 10)) {
        GyroRobot.goForward(speed*1.2);
        Serial.println("goBackward");
    }
    else if (mY >= (constY + 10)) {
        GyroRobot.goBackward(speed*1.2);
        Serial.println("goForward");
    }
    
}   

