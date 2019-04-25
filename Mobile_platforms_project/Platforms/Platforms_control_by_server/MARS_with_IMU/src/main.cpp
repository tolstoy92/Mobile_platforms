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

bool moveSide = true;

// const char* ssid = "SPEECH_405";
// const char* password = "multimodal";
// const char* mqtt_server = "192.168.0.105";

const char* ssid = "iGarage";
const char* password = "igarage18";
const char* mqtt_server = "172.16.30.38";

// const char* ssid = "Redmi";
// const char* password = "qweqweqw";
// const char* mqtt_server = "192.168.43.229";

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
    mqtt.setupWifi();
    mqtt.setCallback(*callback);
    mqtt.subscribe(platformNumber);
    mySensor.setWire(&Wire);
    mySensor.beginMag();
    sensorId = mySensor.readId();

}

void loop()
{
    GyroRobot.enableCentralMotor();
    mqtt.initClientLoop();
    if (correctValue <= 90 && correctValue >= -90) {
        if (moveForwardValue == 1 && rotateValue == 0) {
            if ((mY > (constY - 8)) && (mY < (constY + 10))) {
            GyroRobot.goForward(speed*0.7);
            }
            else if (mY <= (constY - 8)) {
            GyroRobot.goForward(speed*1.2);
            }
            else if (mY >= (constY + 10)) {
            GyroRobot.goForward(speed*0.4);
            }
        }
        if (rotateValue == 0 && moveForwardValue == 0) {
            GyroRobot.stopMovement();
        }
        if (rotateValue == 1 && moveForwardValue == 0)
        {
            GyroRobot.stopMovement();
            if (correctValue > 0) {
                if ((mY > (constY - 8)) && (mY < (constY + 10))) {
                    GyroRobot.turnLeft(speed*0.6);
                }
                else if (mY <= (constY - 8)) {
                    GyroRobot.turnLeft(speed);
                }
                else if (mY >= (constY + 10)) {
                    GyroRobot.turnLeft(speed*0.4);
                }
            }
            else if (correctValue < 0) {
                if ((mY > (constY - 8)) && (mY < (constY + 10))) {
                    GyroRobot.turnRight(speed*0.6);
                }
                else if (mY <= (constY - 8)) {
                    GyroRobot.turnRight(speed);
                }
                else if (mY >= (constY + 10)) {
                    GyroRobot.turnRight(speed*0.4);
                }
            }
        }   
        else if (finishValue == 1) {
            GyroRobot.stopMovement();
        }
    }
    else if (correctValue >= 90 || correctValue <= -90) {
        if (moveForwardValue == 1 && rotateValue == 0) {
            if ((mY > (constY - 10)) && (mY < (constY + 8))) {
            GyroRobot.goBackward(speed*0.7);
            }
            else if (mY >= (constY + 8)) {
            GyroRobot.goBackward(speed*1.2);
            }
            else if (mY <= (constY - 10)) {
            GyroRobot.goBackward(speed*0.4);
            }
        }
        if (rotateValue == 0 && moveForwardValue == 0) {
            GyroRobot.stopMovement();
        }
        if (rotateValue == 1 && moveForwardValue == 0)
        {
           GyroRobot.stopMovement();
           if (correctValue > 0) {
                if ((mY > (constY - 10)) && (mY < (constY + 8))) {
                    GyroRobot.turnRight(speed*0.6);
                }
                else if (mY <= (constY - 10)) {
                    GyroRobot.turnRight(speed*0.4);
                }
                else if (mY >= (constY + 8)) {
                    GyroRobot.turnRight(speed);
                }
            }
            else if (correctValue < 0) {
                if ((mY > (constY - 10)) && (mY < (constY + 8))) {
                    GyroRobot.turnLeft(speed*0.6);
                }
                else if (mY <= (constY - 10)) {
                    GyroRobot.turnLeft(speed*0.4);
                }
                else if (mY >= (constY + 8)) {
                    GyroRobot.turnLeft(speed);
                }
            }
        }   
        else if (finishValue == 1) {
            GyroRobot.stopMovement();
        }    
    }
    //mqtt.pubFeedback(outputData,platformNumber);

    mySensor.magUpdate();
    mY = mySensor.magY();
    mDirection = mySensor.magHorizDirection();
    Serial.println("magY: " + String(mY));
    Serial.println("at " + String(millis()) + "ms");
    Serial.println(""); 
    if (constY == -1000) {
        constY = mY;
    }
    Serial.println(constY);
    // if ((mY > (constY - 8)) && (mY < (constY + 10))) {
    //     GyroRobot.turnLeft(speed*0.6);
    //     Serial.println("Move");
    // }
    // else if (mY <= (constY - 8)) {
    //     GyroRobot.turnLeft(speed);
    //     Serial.println("goBackward");
    // }
    // else if (mY >= (constY + 10)) {
    //     GyroRobot.turnLeft(speed*0.4);
    //     Serial.println("goForward");
    // }
    // GyroRobot.goForward(50);
    // delay(5000);
    // GyroRobot.turnLeft(50);
    // delay(5000);
    // GyroRobot.turnRight(50);
    // delay(5000);
    // GyroRobot.goBackward(50);
    // delay(5000);
    
}
  

