#include <Arduino.h>
#include <Wire.h>
#include <AX12A.h> 
#include "math.h"
#include <I2Cdev.h>
#include <MotorController.h>
#include <MqttClient.h>
#include <PubSubClient.h>
#include <stdio.h>

// hw_timer_t *timer = NULL;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

double duration;

std::string receivedData;
std::string sign;
std::string angle;
std::string move;
std::string rotate;
std::string finish;
std::string valuex;
std::string valuey;

short speed = 70;
short x, y, x1, x2;
short frequency = 500;
short xCoord;
short yCoord;
short correctValue = 0;
short errorValue;
short hallSensorLeft = 0;
short hallSensorRight = 0;

uint8_t finishValue = -1;
uint8_t resolution = 8;
uint8_t splitindex;
uint8_t channel_R = 0;
uint8_t channel_L = 1;
uint8_t rotateValue = -1;
uint8_t moveForwardValue = -1;
uint8_t platformNumber = 7;

// const char* ssid = "mqtt_router";
// const char* password = "multimodal";
// const char* mqtt_server = "192.168.0.18";

const char* ssid = "SPEECH_405";
const char* password = "multimodal";
const char* mqtt_server = "192.168.0.61";

// const char* ssid = "iGarage";
// const char* password = "igarage18";
// const char* mqtt_server = "10.1.30.45";


mqttClient mqtt(ssid, password, mqtt_server);
MotorController Mars;

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
    Mars.setup();
    Mars.setupMotorDriver(frequency, resolution);
    mqtt.setupWifi();
    mqtt.setCallback(*callback);
    mqtt.subscribe(platformNumber);
}

void loop()
{    
    mqtt.initClientLoop();
    if (moveForwardValue == 1 && rotateValue == 0) {
        Mars.moveForward(speed);
    }
    if (rotateValue == 0 && moveForwardValue == 0) {
        Mars.stopMovement();
    }
    if (rotateValue == 1 && moveForwardValue == 0)
    {
       //Mars.stopMovement();
        if (correctValue > 0) {
        Mars.turnLeft(speed);
        }
        else if (correctValue < 0) {
        Mars.turnRight(speed);
        }
    }   
    else if (finishValue == 1) {
    Mars.stopMovement();
    }
    //mqtt.pubFeedback(outputData,platformNumber);
}   

