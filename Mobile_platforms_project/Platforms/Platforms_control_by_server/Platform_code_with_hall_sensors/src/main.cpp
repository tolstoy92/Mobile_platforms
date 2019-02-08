#include <Arduino.h>
#include <MotorController.h>
#include <SPI.h>
#include "esp32-hal-ledc.h"
#include <MqttClient.h>
#include <PubSubClient.h>
#include "math.h"

#define PIN_ENABLE_R 13
#define PIN_FORWARD_R 12
#define PIN_BACK_R 14
#define PIN_FORWARD_L 27
#define PIN_BACK_L 26
#define PIN_ENABLE_L 25
#define PIN_HALL_R 32
#define PIN_HALL_L 33

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile int interruptCounter;
int totalInterruptCounter = 0;
int encoderRightCnt = 0;
int encoderLeftCnt = 0;

float minSpeed = 0;
float maxSpeed = 1;
float minControlSignal = 100;
float maxControlSignal = 200;
float maxError = 0.5;
float rightSpeed = 0;
float leftSpeed = 0;
float controlSignal;

float kp = 3;
float kd = 0.2;
float ki = 0.3;

float errorRightWheel;
float errorLeftWheel;
float integral;
float prevError;
float minIntegral = -30;
float maxIntegral = 30;
float targetSpeed = 0.5;
float reduceSpeed = 5;
float reduceSpeedSide = 5;
float yRight;
float yLeft;
float uRight;
float uLeft;

std::string receivedData;
std::string sign;
std::string angle;
std::string move;
std::string rotate;
std::string finish;
std::string valuex;
std::string valuey;

short x, y, x1, x2;
short frequency = 500;
short xCoord;
short yCoord;
short correctValue = 0;
short errorValue;
short hallSensorLeft = 0;
short hallSensorRight = 0;

uint8_t topic_id = 2;
uint8_t finishValue = -1;
uint8_t resolution = 10;
uint8_t splitindex;
uint8_t channel_R = 0;
uint8_t channel_L = 1;
uint8_t rotateValue = -1;
uint8_t moveForwardValue = -1;
uint8_t platformNumber = 2;

// const char* ssid = "SPEECH_405";
// const char* password = "multimodal";
// const char* mqtt_server = "192.168.0.105";

const char* ssid = "iGarage";
const char* password = "igarage18";
const char* mqtt_server = "10.1.30.38";

// const char* ssid = "service_iG";
// const char* password = "1213141516igarage";
// const char* mqtt_server = "192.168.1.65";

mqttClient mqtt(ssid, password, mqtt_server);

MotorController DriveCar;

void IRAM_ATTR onTimer() 
{

    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
 
}

void init_Timer() 
{  
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000, true);
    timerAlarmEnable(timer);
}

void doEncoderRight()
{ 
    static byte pred_r = 0;
    byte r = digitalRead(PIN_HALL_R);
    if(r != pred_r)
    {
        pred_r = r;
        encoderRightCnt++;
    }
}

void doEncoderLeft()
{ 
    static byte pred_l = 0;
    byte l = digitalRead(PIN_HALL_L);
    if(l != pred_l)
    {
        pred_l = l;
        encoderLeftCnt++;
    }
}

void anglePid()
{

}

void callback(char* topic, byte* message, unsigned int length)
{
    char angleTopic[64];
    char finishTopic[64];
    char moveTopic[64];
    char rotateTopic[64];

    sprintf(angleTopic, "platforms/%d", platformNumber);
    sprintf(finishTopic, "platforms/%d/on_finish", platformNumber);
    sprintf(moveTopic, "platforms/%d/move", platformNumber);
    sprintf(rotateTopic, "platforms/%d/rotate", platformNumber);

    if (strcmp(topic, angleTopic)==0) {

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
            //correctValue = atoi(receivedData.c_str());   
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
        Serial.println(correctValue);
        Serial.println(moveForwardValue);
        Serial.println(rotateValue );
        Serial.println(finishValue);
    }

    // if ((strcmp(topic, moveTopic)==0)) {

    //     receivedData = "";

    //     for (int i = 0; i < length; i++)
    //     {
    //         receivedData += (char)message[i];
    //         moveForwardValue = atoi(receivedData.c_str()); 
    //     }
    //     Serial.println(moveForwardValue);
         
    // }

    // if ((strcmp(topic, finishTopic)==0)) {

    //     receivedData = "";

    //     for (int i = 0; i < length; i++)
    //     {
    //         receivedData += (char)message[i];
    //         finishValue = atoi(receivedData.c_str()); 
    //     }
    //     Serial.println(finishValue);
         
    // }

    // if (strcmp(topic, rotateTopic)==0) {

    //     receivedData = "";

    //     for (int i = 0; i < length; i++)
    //     {
    //         receivedData += (char)message[i];
    //         rotateValue = atoi(receivedData.c_str());    
    //     }
    //     Serial.println(rotateValue);
    // }
    
    
}

float yPidControl(float yCorrectValue)
{
    float y;
    integral = integral + yCorrectValue;
    if(integral>maxIntegral) integral=maxIntegral;
    if(integral<minIntegral) integral=minIntegral;
    float rdiff = kd*(yCorrectValue - prevError);
    y = (kp*yCorrectValue + ki*integral + rdiff);
    prevError = yCorrectValue; 
    return y;
}

float uPidControl(float uCorrectValue)
{
    float u = 0;
    float maxY = kp*maxError + ki*maxIntegral + kd*maxError;
    float minY = -maxY;
    u = ((uCorrectValue-minY)/(maxY-minY))*((maxControlSignal-minControlSignal))+minControlSignal;
    return u;
}

void setup()
{
    Serial.begin(115200);
    mqtt.setupWifi();
    mqtt.setCallback(*callback);
    init_Timer();
    DriveCar.setup(PIN_ENABLE_R,PIN_FORWARD_R,PIN_BACK_R,PIN_FORWARD_L,PIN_BACK_L,PIN_ENABLE_L, channel_R, channel_L);
    DriveCar.setupMotorDriver(channel_R, channel_L, frequency, resolution);
    pinMode(PIN_HALL_L, INPUT);
    pinMode(PIN_HALL_R, INPUT);
}

void driveMotor(short correctValue)
{
    DriveCar.controlByCamera(correctValue, reduceSpeed, reduceSpeedSide);
}

void loop()
{
    while (finishValue != 1) {
        mqtt.initClientLoop();
        mqtt.subscribe(topic_id);

        if (moveForwardValue == 1 && rotateValue == 0) {
            
            doEncoderRight();
            doEncoderLeft();

            if (interruptCounter > 0) {

                portENTER_CRITICAL(&timerMux);
                interruptCounter--;
                rightSpeed = encoderRightCnt;
                leftSpeed = encoderLeftCnt;
                encoderRightCnt = 0;
                encoderLeftCnt = 0;
                portEXIT_CRITICAL(&timerMux);

                totalInterruptCounter++;

                Serial.println(rightSpeed/6);
                Serial.println(leftSpeed/6);

                errorRightWheel = targetSpeed - rightSpeed/6;
                errorLeftWheel = targetSpeed - leftSpeed/6;

                yRight = yPidControl(errorRightWheel);
                yLeft = yPidControl(errorLeftWheel);

                uRight = uPidControl(yRight);
                uLeft = uPidControl(yLeft);

                //Serial.println(yRight);
                //Serial.println(yLeft);

                //Serial.println(uRight);
                //Serial.println(uLeft);

                DriveCar.moveForward(uRight, uLeft);
                //Serial.println(uRight);
            }
        }

        if (rotateValue == 0 && moveForwardValue == 0) {

            DriveCar.stop(correctValue);
        }
     
        if (rotateValue == 1 && moveForwardValue == 0)
        {
            DriveCar.stop(correctValue);
           
            if (correctValue > 0) {
            DriveCar.rotateLeft(correctValue);
            }
            else if (correctValue < 0) {
            DriveCar.rotateRight(correctValue);
            }
        }   
    }
}   
   


    
