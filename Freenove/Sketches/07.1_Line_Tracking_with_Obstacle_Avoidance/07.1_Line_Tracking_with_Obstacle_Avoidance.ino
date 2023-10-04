/**********************************************************************
  Filename    : Tracking_Car.ino
  Product     : Freenove 4WD Car for ESP32
  Auther      : www.freenove.com
  Modification: 2020/12/18
**********************************************************************/

#include <Arduino.h>
#include "Freenove_4WD_Car_For_ESP32.h"

#define SPEED_LV4   ( 4000 )
#define SPEED_LV3   ( 3000 )
#define SPEED_LV2   ( 2500 )
#define SPEED_LV1   ( 1500 )

void setup()
{
  Track_Setup();   //Trace module initialization
  Serial.begin(115200);//Open the serial port and set the baud rate to 115200
  PCA9685_Setup(); //Motor drive initialization
  Ultrasonic_Setup();//Initialize the ultrasonic module
  Servo_1_Angle(90);   //Set the initial value of Servo 1 to 90 degrees
  Servo_2_Angle(90);   //Set the initial value of Servo 2 to 90 degrees
  Emotion_Setup();
}

void loop()
{
  Track_Read();
        Servo_2_Angle(90);  
        Servo_1_Angle(90);
        delay(10);

    Serial.print("Distance: " + String(DetectObstacle()) + "\n");

  switch (sensorValue[3])
  {
    case 2:   //010
    case 5:   //101
    case 7:   //111
      showArrow(1, 100);
      Motor_Move(SPEED_LV1, SPEED_LV1, SPEED_LV1, SPEED_LV1);    //Move Forward
      break;
    case 0:   //000
      eyesBlink1(100);
      Motor_Move(0, 0, 0, 0);                                    //Stop
      break;
    case 4:   //100
    case 6:   //110
      wheel(2, 100);
      Motor_Move(SPEED_LV4, SPEED_LV4 , - SPEED_LV4, -SPEED_LV4);//Turn Right
      break;
    case 1:   //001
    case 3:   //011
      wheel(1, 100);
      Motor_Move(-SPEED_LV4, -SPEED_LV4, SPEED_LV4, SPEED_LV4);  //Turn Left
      break;
    default:
      break;
    case 8: 
      Motor_Move(-SPEED_LV1, -SPEED_LV1, -SPEED_LV1, -SPEED_LV1);  //Move Backward
      break;
  }
}

