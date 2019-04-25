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

// char outputData[10] = "hi";
char *data;

short constY = -1000;

int mZcount = 0;

short speed = 100;
short correctValue = 0;
short infraredSignal = 0;

uint8_t finishValue = -1;
uint8_t splitindex;
uint8_t rotateValue = -1;
uint8_t moveForwardValue = -1;
uint8_t platformNumber = 3;
uint8_t sensorId;
uint8_t infraredPin = 23;
uint8_t infraredSensorPin = 15;
uint8_t radix = 10;
char buffer[20];


const char* ssid = "SPEECH_405";
const char* password = "multimodal";
const char* mqtt_server = "192.168.0.193";

// const char* ssid = "iGarage";
// const char* password = "igarage18";
// const char* mqtt_server = "10.1.30.45";

mqttClient mqtt(ssid, password, mqtt_server);

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
    Serial.begin(115200);
    mqtt.setupWifi();
    mqtt.setCallback(*callback);
    mqtt.subscribe(platformNumber);
    pinMode(infraredSensorPin, INPUT);
    pinMode(infraredPin, OUTPUT);
}

void loop()
{
    mqtt.initClientLoop();
    digitalWrite(infraredPin, HIGH);
    infraredSignal = analogRead(infraredSensorPin);
    Serial.println(infraredSignal);
    data = itoa(infraredSignal, buffer, radix);
    mqtt.pubFeedback(data,platformNumber);
    delay(300);
}

  

