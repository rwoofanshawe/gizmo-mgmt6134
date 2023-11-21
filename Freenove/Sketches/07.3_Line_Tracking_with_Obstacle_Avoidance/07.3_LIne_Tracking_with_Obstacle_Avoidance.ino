/**********************************************************************
  Filename    : Tracking_Car.ino
  Product     : Freenove 4WD Car for ESP32
  Auther      : www.freenove.com
  Modification: 2020/12/18
**********************************************************************/

#include <Arduino.h>
#include "Freenove_4WD_Car_For_ESP32.h"

float suitabledistance[3];
int leftmotor[4];                   // stores 2 motor and its speed on the left side of the car
int rightmotor[4];                  // stores 2 motor and its speed on the right side of the car
float angle[3];                       // stores the angle degree of the scanner
int faremotion[3];
int nearemotion[3];
int motor[5];                       // stores the speed of the 4 motors
int oppositemotor[5];               // stores the opposite speed of the 4 motors
int turndistance[4];                // stores distance limit for moving away to the obstacle
int sensorcondition[3];             // stores condition of the sensor to return to track
//RT speed of the motor for return to track
int RTleftmotor[3];
int RTrightmotor[3];
int RTmotor[5]; 
int RToppositemotor[5]; 

float distance[4];              //Storage of ultrasonic data

int SDcounter;
int NFcounter;
int counter;


//Calibration

//turn line tracking
int SPEED_LV2=1000;               //left and right from line tracking; decrease if car is wiggling at the track
int SPEED_LV3=1100;
//forard line tracking
int SPEED_LV1=580;                //forward from line tracking; decrease if car cannot turn to curve
//ahead
int ahead=14;             //distance detecting obstacle from line trackiing; or far distance car will move around the obstacle
//suitabledistance
float sdistance=4.5;                //distance from osbatcle before moving around; or near distance car will move around the obstacle
//forward speed suitable distance and forward to zero
int SPEED_LV4=550;
//obstacleavoidance
int SDdelay=120;                  //time for car to turn once obstacle is detected
//turnnearfar
int NFdelay=10;                    //time for car to turn near or far from obstacle when it moves around the obstacle
//speed around the osbtacle
int SPEED_LV5=1200;
//reterntotrack
int RTdelay=50;                  //time for car to return to track

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
  if (sensorValue[3] >= 1 && sensorValue[3] <= 7)     //Gizmo is on line track
  { 
    if (ObstacleAhead()==1)                           // 1 obstacle is detected
    {
      ObstacleAvoidance();
    } else {
      /*if (!(SPEED_LV1 == 1500)){
        SPEED_LV1=SPEED_LV1+1;
      }
      if (!(SPEED_LV2 == 2500)){
        SPEED_LV2=SPEED_LV2+1;
      }*/
      LineTracking();
    }
  } else {
  Servo_1_Angle(90);//Set the Angle value of servo 2 to 90°
  }
}

void LineTracking()
{
  Track_Read();
  switch (sensorValue[3])
  {
    case 2:   //010
    case 5:   //101
    case 7:   //111
      showArrow(1, 100);
      Motor_Move(SPEED_LV1, SPEED_LV1, SPEED_LV1, SPEED_LV1);    //Move Forward
      break;
    case 0:   //000
      eyesCry(100);
      Motor_Move(0, 0, 0, 0);                                  //Stop
      break;
    case 4:   //100
    case 6:   //110
      wheel(2, 100);
      Motor_Move(SPEED_LV2, SPEED_LV2 , -SPEED_LV3, -SPEED_LV3);//Turn Right
      break;
    case 1:   //001
    case 3:   //011
      wheel(1, 100);
      Motor_Move(-SPEED_LV3, -SPEED_LV3, SPEED_LV2, SPEED_LV2);  //Turn Left
      break;
    default:
      break;
  }
}

void get_distance(void)
{   
  /*Servo_2_Angle(90);  
  Servo_1_Angle(120);
  delay(100); */                                          
  distance[0] = Get_Sonar();//Get the data on the left

  Servo_1_Angle(90);
  //delay(100);
  distance[1] = Get_Sonar();//Get the data in the in front
/*
  Servo_1_Angle(60);
  delay(100); */
  distance[2] = Get_Sonar();//Get the data on the right
/*
  Servo_1_Angle(90);
  delay(100);*/
  distance[1] = Get_Sonar();//Get the data in the in front

}

int ObstacleAhead(void)
{
  int obstacle=0;
  get_distance();
  if ((distance[0] < ahead) || (distance[1] < ahead) || (distance[2] < ahead))
  {
    eyesBlink1(100);
    Motor_Move(0, 0, 0, 0);                               //Stop
    obstacle=1;                                           //Obstacle detected
  }
  else
    obstacle=0;                                           //No obstacle detected
  return obstacle;
}

void movement(int turn)
{
  leftmotor[3]=-SPEED_LV5; 
  rightmotor[3]=SPEED_LV5;
  turndistance[3]=((ahead-sdistance)/2);                 // increase this value if it gets too close to the obstacke when moving around decrease if it is too far
  RTleftmotor[3]=-SPEED_LV3;
  RTrightmotor[3]=SPEED_LV3;

  //1 left direction
  leftmotor[1]=leftmotor[3];
  rightmotor[1]=rightmotor[3];
  angle[1]=0;       // scanner on the right
  turndistance[1]=turndistance[3];
  sensorcondition[1]=3;
  RTleftmotor[1]=RTleftmotor[3];
  RTrightmotor[1]=RTrightmotor[3];
  faremotion[1]=3;
  nearemotion[1]=4;

  //2 right direction
  leftmotor[2]=rightmotor[3];
  rightmotor[2]=leftmotor[3];
  angle[2]=176;   // scanner on the left
  turndistance[2]=turndistance[3]+1;
  sensorcondition[2]=6;
  RTleftmotor[2]=RTrightmotor[3];
  RTrightmotor[2]=RTleftmotor[3];
  faremotion[2]=4;
  nearemotion[2]=3;

  //motor speed to get far the obstacle
  motor[1]=leftmotor[turn];
  motor[2]=leftmotor[turn];
  motor[3]=rightmotor[turn];
  motor[4]=rightmotor[turn];
  RTmotor[1]=RTleftmotor[turn];
  RTmotor[2]=RTleftmotor[turn];
  RTmotor[3]=RTrightmotor[turn];
  RTmotor[4]=RTrightmotor[turn];

  //motor speed to get near the obstacle
  oppositemotor[1]=rightmotor[turn];
  oppositemotor[2]=rightmotor[turn];
  oppositemotor[3]=leftmotor[turn];
  oppositemotor[4]=leftmotor[turn];
  RToppositemotor[1]=RTrightmotor[turn];
  RToppositemotor[2]=RTrightmotor[turn];
  RToppositemotor[3]=RTleftmotor[turn];
  RToppositemotor[4]=RTleftmotor[turn];

  //angle degree of the scanner
  angle[0]=angle[turn];
  faremotion[0]=faremotion[turn];
  nearemotion[0]=nearemotion[turn];
  turndistance[0]=turndistance[turn];
  sensorcondition[0]=sensorcondition[turn];
}

void ObstacleAvoidance(void)
{
  suitabledistance[1]=sdistance;
  suitabledistance[2]=suitabledistance[1]+1;

  SuitableDistance();             // Gizmo moves on a suitable distance from the obstacle

  movement(ObstacleLocation());   // determine the location of the obstcle then proceed with the right direction

  Servo_1_Angle(angle[0]);        // scanner moves to the right degree

  SDcounter=0;
  while (true) {
    if (SDcounter < SDdelay){
      showArrow(faremotion[0], 100);
      Motor_Move(motor[1], motor[2], motor[3], motor[4]);
      SDcounter=SDcounter+1;
    } else {
      eyesBlink1(100);
      Motor_Move(0, 0, 0, 0);                           //Stop
      break; // Exit the loop when sonarValue is 20 or higher
    }
  }
  MoveAroundObstacle();
}

void SuitableDistance(void)
{/*
  if ((distance[0] < distance[2]) && (distance[0] < distance[1])){
    Servo_1_Angle(120);
  } else if ((distance[2] < distance[0])  && (distance[2] < distance[1])){
    Servo_1_Angle(60);
  } else {
    Servo_1_Angle(90);
  }
*/
  while (true) {
    float sonarValue = Get_Sonar();

    if (sonarValue < suitabledistance[1]) {
      showArrow(2, 100);
      Motor_Move(-SPEED_LV4, -SPEED_LV4, -SPEED_LV4, -SPEED_LV4);                   //backward 
    } else if (sonarValue > suitabledistance[2]) {
      showArrow(1, 100);
      Motor_Move(SPEED_LV4, SPEED_LV4, SPEED_LV4, SPEED_LV4);               //Forward
    } else {
      eyesBlink1(100);
      Motor_Move(0, 0, 0, 0);                           //Stop
      break; // This will exit the loop
    }
  }
}

int ObstacleLocation(void)
{
  int location=2;
/*
  get_distance();

  if (distance[0] < distance[2]) {
    location=1;                                      //1 is left                                    
  } else {
    location=2;                                     //2 is right
  }
*/
  return location;
}

void MoveAroundObstacle(void)
{
  while (true) {
    Track_Read();
    if (sensorValue[3] == 0) {
      MoveForward();
      Track_Read();
      if (sensorValue[3] == 0) {
        float sonarValue = Get_Sonar();

        if (sonarValue > turndistance[0]) {
          MoveNearObstacle();
        } else {
          MoveFarObstacle();
        }
      } else {
        ReturnTrack();
        break; // Exit the loop when sensorValue[3] is not equal to 0
      }
    } else {
      ReturnTrack();
      break; // Exit the loop when sensorValue[3] is not equal to 0
    }
  }
}

void MoveForward(void)
{
  while (true) {   
    Track_Read();
    if (!(sensorValue[3] == 7)) {
      float sonarValue = Get_Sonar();
      if ((sonarValue > sdistance-2) && (sonarValue < ahead-3)) {
        showArrow(1, 100);
        Motor_Move(rightmotor[3], rightmotor[3], rightmotor[3], rightmotor[3]);
      } else {
        eyesBlink1(100);
        Motor_Move(0, 0, 0, 0);                           //Stop
        break; // Exit the loop when sonarValue is 7 or higher
      } 
    } else {
      eyesSmile(100);
      Motor_Move(0, 0, 0, 0);                           //Stop
      break; // Exit the loop when sonarValue is 7 or higher
    }
  }
}

void MoveNearObstacle(void)
{
  NFcounter=0;
  while (true) {   
    Track_Read();
    if (!(sensorValue[3] == 7)) {
      if (NFcounter < NFdelay){
        showArrow(nearemotion[0], 100);
        Motor_Move(oppositemotor[1], oppositemotor[2], oppositemotor[3], oppositemotor[4]);
        NFcounter=NFcounter+1;
      } else {
        eyesBlink1(100);
        Motor_Move(0, 0, 0, 0);                           //Stop
        break; // Exit the loop when sonarValue is 20 or higher
      }
    } else {
      eyesSmile(100);
      Motor_Move(0, 0, 0, 0);                           //Stop
      break; // Exit the loop when sonarValue is 20 or higher
    }
  } 
}

void MoveFarObstacle(void)
{
  NFcounter=0;
  while (true) {   
    Track_Read();
    if (!(sensorValue[3] == 7)) {
      if (NFcounter < NFdelay){
        showArrow(faremotion[0], 100);
        Motor_Move(motor[1], motor[2], motor[3], motor[4]);
        NFcounter=NFcounter+1;
      } else {
        eyesBlink1(100);
        Motor_Move(0, 0, 0, 0);                           //Stop
        break; // Exit the loop when sonarValue is 20 or higher
      }
    } else {
      eyesSmile(100);
      Motor_Move(0, 0, 0, 0);                           //Stop
      break; // Exit the loop when sonarValue is 20 or higher
    }
  } 
}

void ReturnTrack(void)
{
  Servo_1_Angle(90);

  ForwardToZero();
  counter=0;
  while (true) {
    if (counter < RTdelay) {
      TurnToTrack();
      Track_Read();
      if (counter < RTdelay) {
        ForwardToTrack();
      } else {
        break; // Exit the loop when counter is less than to 100
      }
    } else {
      break;
    }
  }
/*
  SPEED_LV1=800;
  SPEED_LV2=1000;
*/
}

void ForwardToZero(void)
{   
  while (true){
    Track_Read();
    if (!(sensorValue[3] == 0)) {
      showArrow(1, 100);
      Motor_Move(SPEED_LV4, SPEED_LV4, SPEED_LV4, SPEED_LV4);
    } else {
      eyesSmile(100);
      Motor_Move(0, 0, 0, 0);                          //Stop
      break; // Exit the loop when sonarValue is not 0
    }
  }    
}

void TurnToTrack(void)
{
  while (true){
    Track_Read();
    if (!(sensorValue[3] == 7)) {
      showArrow(faremotion[0], 100);
      Motor_Move(RTmotor[1], RTmotor[2], RTmotor[3], RTmotor[4]);
    } else {
      eyesSmile(100);
      Motor_Move(0, 0, 0, 0);                           //Stop
      break; // Exit the loop when sonarValue is not equal to 7 
    }
  }  
  while (true){
    Track_Read();
    if (!(sensorValue[3] == 7)) {
      showArrow(nearemotion[0], 100);
      Motor_Move(RToppositemotor[1], RToppositemotor[2], RToppositemotor[3], RToppositemotor[4]);
    } else {
      eyesSmile(100);
      Motor_Move(0, 0, 0, 0);                           //Stop
      break; // Exit the loop when sonarValue is not equal to 7 
    }
  }        
}

void ForwardToTrack(void)
{   
  if (!(sensorValue[3] == 7))
    TurnToTrack();

  counter=0;
  while (true){
    Track_Read();
    if (!(sensorValue[3] == 0)) {
      if (counter < RTdelay) {
        Track_Read();
        if (!(sensorValue[3] == sensorcondition[0])) {
          counter=counter+1;
          showArrow(1, 100);
          Motor_Move(SPEED_LV1, SPEED_LV1, SPEED_LV1, SPEED_LV1);
        } else {
          eyesSmile(100);
          Motor_Move(0, 0, 0, 0);                         //Stop
          break; // Exit the loop when sonarValue is not equal to sensorcondition
        }
      } else {
        eyesSmile(100);
        Motor_Move(0, 0, 0, 0); 
        break;
      }
    } else {
      counter=2000;
      eyesSmile(100);
      Motor_Move(0, 0, 0, 0); 
      break;
    }
  }   
}

