/**********************************************************************
  Filename    : Photosensitive_Car.ino
  Product     : Freenove 4WD Car for ESP32
  Auther      : www.freenove.com
  Modification: 2020/12/18
**********************************************************************/
#include <Arduino.h>
#include "Freenove_4WD_Car_For_ESP32.h"

int photosensitive_sensitivity = 100;//Set the sensitivity of the photosensitive resistor trolley
int photosensitive_init_value = 0;   //Set the car's initial environment ADC value

void setup()
{
  Track_Setup();   //Trace module initialization
  Serial.begin(115200);//Open the serial port and set the baud rate to 115200
  PCA9685_Setup(); //Motor drive initialization
  Ultrasonic_Setup();//Initialize the ultrasonic module
  Photosensitive_Setup();  //Photosensitive initialization
  Servo_1_Angle(90);   //Set the initial value of Servo 1 to 90 degrees
  Servo_2_Angle(90);   //Set the initial value of Servo 2 to 90 degrees
  Emotion_Setup();
  Buzzer_Setup();
  WS2812_Setup();
  photosensitive_init_value = Get_Photosensitive();
}

int SPEED_LV6=1000;                //forward and backward
int SPEED_LV7=1400;               //turn

int initial=0;
int value=255;

int Lcounter=0;

void loop()
{
int left=photosensitive_init_value-1400;          // minimum range to move forward and if its lower than this car turns left
int frontleft=photosensitive_init_value-600;      // maximum range to move forward and if its higher than this car stops (minimum range to stop)
int frontright=photosensitive_init_value+600;     // minimum range to move backward and if its lower than this car stops (maximum range to stop)
int right=photosensitive_init_value+1800;         // maximum range to move backward and if its higher than this car turns right

  initial = value;

  if (Lcounter < 500) {
     if (initial = 255) {
      photosensitive_init_value = Get_Photosensitive();
      Lcounter=Lcounter+1;
    } else {
      value = 0;
    }
  } else {
    value = 0;
  }

  WS2812_Set_Color_1(4095, initial, initial, initial);
  WS2812_Show(1);

  //There is a light source both on the left just enough to move forward
  if ((Get_Photosensitive() > left) && (Get_Photosensitive() < frontleft))
  {
    showArrow(1, 100);
    Motor_Move(SPEED_LV6, SPEED_LV6, SPEED_LV6, SPEED_LV6);
  }
  //There is a light source both on the right just enough to move backward
  else if ((Get_Photosensitive() > frontright) && (Get_Photosensitive() < right))
  {
    showArrow(2, 100);
    Motor_Move(-SPEED_LV6, -SPEED_LV6, -SPEED_LV6, -SPEED_LV6);
  }
  //There is a light source on the left side of the car
  else if (Get_Photosensitive() < left)
  {
    wheel(1,100);
    Motor_Move(-SPEED_LV7, -SPEED_LV7, SPEED_LV7, SPEED_LV7);
  }
  //There is a light source on the right side of the car
  else if (Get_Photosensitive() > right)
  {
    wheel(2,100);
    Motor_Move(SPEED_LV7, SPEED_LV7, -SPEED_LV7, -SPEED_LV7);
  }
  //The light is in the middle of the car
  else
  {
    eyesCry(100);
    Motor_Move(0, 0, 0, 0);
  } 
}
