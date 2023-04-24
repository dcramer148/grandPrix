#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/LPF.h"
#include "../inc/opt3101.h"
#include "../inc/LaunchPad.h"
#include "../inc/Bump.h"
//#include "../inc/Motor.h"
#include "../inc/UART0.h"
#include "../inc/SSD1306.h"
#include "../inc/FFT.h"
#include "../inc/motorDriver.h"
#include "../inc/odometry.h"
#include "../inc/Tachometer.h"
#include "../inc/SysTickInts.h"
#include "../inc/TimerA1.h"

// Select one of the following three output possibilities
// define USENOKIA
#define USEOLED 1
//#define USEUART

#ifdef USENOKIA
// this batch configures for LCD
#include "../inc/Nokia5110.h"
#define Init Nokia5110_Init
#define Clear Nokia5110_Clear
#define SetCursor Nokia5110_SetCursor
#define OutString Nokia5110_OutString
#define OutChar Nokia5110_OutChar
#define OutUDec Nokia5110_OutUDec
#define OutSDec Nokia5110_OutSDec
#endif

#ifdef USEOLED
// this batch configures for OLED

void OLEDinit(void){SSD1306_Init(SSD1306_SWITCHCAPVCC);}
#define Init OLEDinit
#define Clear SSD1306_Clear
#define SetCursor SSD1306_SetCursor
#define OutChar SSD1306_OutChar
#define OutString SSD1306_OutString
#define OutUDec SSD1306_OutUDec
#define OutSDec SSD1306_OutSDec
#endif

#ifdef USEUART
// this batch configures for UART link to PC
#include "../inc/UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY){
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec
#define OutSDec UART0_OutSDec
#endif


uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t Noises[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec
bool pollDistanceSensor(void){
  if(OPT3101_CheckDistanceSensor()){
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}

// calibrated for 500mm track
// right is raw sensor data from right sensor
// return calibrated distance from center of Robot to right wall
int32_t Right(int32_t right){
  return  (right*(59*right + 7305) + 2348974)/32768;
}
// left is raw sensor data from left sensor
// return calibrated distance from center of Robot to left wall
int32_t Left(int32_t left){
  return (1247*left)/2048 + 22;
}
// assumes track is 500mm
int32_t Mode=0; // 0 stop, 1 run
int32_t Error;
int32_t Error_prior = 0;
int32_t Ki= 0;  // integral controller gain
int32_t Kp= 4;  // proportional controller gain //was 4  //Decent values => Kd = 200, Kp = 50 or both 100
int32_t Kd = 100; //Derivative
int32_t UR, UL;  // PWM duty 0 to 14,998

#define TOOCLOSE 200 //was 200
#define DESIRED 250 //was 250
int32_t SetPoint = 250; // mm //was 250
int32_t LeftDistance,CenterDistance,RightDistance; //mm
#define TOOFAR 400 // was 400

#define PWMNOMINAL 7500 // was 2500 -> 7500
#define SWING 1000 //was 1000
#define PWMMIN (PWMNOMINAL-SWING)
#define PWMMAX (PWMNOMINAL+SWING)

void Controller(void){ // runs at 100 Hz
  if(Mode){
    if((LeftDistance>DESIRED)&&(RightDistance>DESIRED)){
      SetPoint = (LeftDistance+RightDistance)/2;
    }else{
      SetPoint = DESIRED;
    }
    if(LeftDistance < RightDistance ){
      Error = LeftDistance-SetPoint;
    }else {
      Error = SetPoint-RightDistance;
    }
 //   UR = UR + Ki*Error;      // adjust right motor
    UR = PWMNOMINAL+Kp*Error; // proportional control
    UL = PWMNOMINAL-Kp*Error; // proportional control
    if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
    if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;
    //Motor_Forward(UL,UR);

  }
}

void Controller_Right(void){ // runs at 100 Hz
  if(Mode){
    if((RightDistance>DESIRED)){
      SetPoint = (RightDistance)/2;
    }else{
      SetPoint = DESIRED;
    }
    /*if(LeftDistance < RightDistance ){
      Error = LeftDistance-SetPoint;
    }else {
      Error = SetPoint-RightDistance;
    }*/

    Error = SetPoint - RightDistance;
    //UL = UL + Ki*Error;      // adjust right motor
    UR = PWMNOMINAL+Kp*Error; // proportional control
    UR = UR + Kd*(Error-Error_prior); //derivative control
    UR = UR + Ki*Error;      // adjust right motor
    UL = PWMNOMINAL-Kp*Error; // proportional control

    Error_prior = Error;

    if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
    if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;

    //turns left if the center measurement and right measurement is small enough that we will hit the wall if we don't turn
    if((RightDistance < 250) && (CenterDistance < 250)){
        UL = 0;
        UR = PWMNOMINAL;
    }
    //Turn left
   /* if(RightDistance < TOOCLOSE){
        UL = PWMNOMINAL/2;
        UR = PWMNOMINAL;
    }*/

    //turns left if the center measurement is small enough that we will hit the wall if we don't turn
    if(CenterDistance < 250){
        UL = 0;
        UR = PWMNOMINAL;
    }
    //Slight Left
    else if(CenterDistance < 300){
        UL = PWMNOMINAL/2;
        UR = PWMNOMINAL;
    }
    //Sharper Turns - Turn Right
    if ((RightDistance > TOOFAR+50) && (CenterDistance > TOOFAR)){
        UL = PWMNOMINAL;
        UR = PWMNOMINAL/2;
    }
    //Hard Left
    if(LeftDistance < 275){ //200 min
        UL = 0;
        UR = PWMMAX;
    }

    PWM_RightMotor(UR);
    PWM_LeftMotor(UL);
  }
}

void Pause(void){int i;
  while(Bump_Read() != 0xED){ // wait for release
    Clock_Delay1ms(200); LaunchPad_Output(0); // off
    Clock_Delay1ms(200); LaunchPad_Output(1); // red
  }
  while(Bump_Read() == 0xED){// wait for touch
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(3); // red/green
  }
  while(Bump_Read() != 0xED){ // wait for release
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(4); // blue
  }
  for(i=1000;i>100;i=i-200){
    Clock_Delay1ms(i); LaunchPad_Output(0); // off
    Clock_Delay1ms(i); LaunchPad_Output(2); // green
  }
  // restart Jacki
  UR = UL = PWMNOMINAL;    // reset parameters
  Mode = 1;

}

//For tachometry
/*
enum RobotState Action;
uint16_t LeftTach;             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;    // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;             // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
int32_t LastLeftSteps;         // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
int32_t TotalLeftSteps;        // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach;            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;   // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;            // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
int32_t LastRightSteps;        // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
int32_t TotalRightSteps;       // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
int32_t MyX,MyY;               // position in 0.0001cm
int32_t MyTheta;               // direction units 2*pi/16384 radians (-pi to +pi)
int32_t Error; // in 0.0001cm or in 2*pi/16384 radians
*/
//For odometry
extern int32_t MyX,MyY;               // position in 0.0001cm
extern int32_t MyTheta;               // direction units 2*pi/16384 radians (-pi to +pi)
extern enum RobotState Action;
uint32_t result;

//For tachometry
uint16_t avg(uint16_t *array, int length)
{
  int i;
  uint32_t sum = 0;

  for(i=0; i<length; i=i+1)
  {
    sum = sum + array[i];
  }
  return (sum/length);
}
int32_t t;
uint16_t ActualL;                        // actual rotations per minute
uint16_t ActualR;
#define TACHBUFF 10                      // number of elements in tachometer array
uint16_t LeftTach2[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir2;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps2;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach2[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir2;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps2;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)


void main(void){ // wallFollow wall following implementation
  int i = 0;
  int j = 0;
  t = 0;
  uint32_t channel = 1;
  uint32_t xPos,yPos,headAngle;
  xPos = 0;
  yPos = 0;
  headAngle = 0;
  DisableInterrupts();
  Clock_Init48MHz();
  UART0_Initprintf(); // BAUD Rate = 115,200 bps
  printf("\n\rOdometry Test\n\r");
  Bump_Init();
  LaunchPad_Init(); // built-in switches and LEDs
  motorPWMInit(15000,0,0);
  Odometry_Init(0,0,NORTH);
  Tachometer_Init();
  TimerA1_Init(&UpdatePosition,20000); // every 40ms
  Mode = 1;
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  Init();
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
  TxChannel = 3;
  OPT3101_StartMeasurementChannel(channel);
  LPF_Init(100,8);
  LPF_Init2(100,8);
  LPF_Init3(100,8);
  UR = UL = PWMNOMINAL; //initial power
  //Pause();
  EnableInterrupts();

  while(1){
 /* if(Bump_Read() != 0xED){ // collision
       Mode = 0;
       Motor_Stop();
       Pause();
    }*/
    if(TxChannel <= 2){ // 0,1,2 means new data
      if(TxChannel==0){
        if(Amplitudes[0] > 1000){
          LeftDistance = FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
        }else{
          LeftDistance = FilteredDistances[0] = 500;
        }
      }else if(TxChannel==1){
        if(Amplitudes[1] > 1000){
          CenterDistance = FilteredDistances[1] = LPF_Calc2(Distances[1]);
        }else{
          CenterDistance = FilteredDistances[1] = 500;
        }
      }else {
        if(Amplitudes[2] > 1000){
          RightDistance = FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
        }else{
          RightDistance = FilteredDistances[2] = 500;
        }
      }
      TxChannel = 3; // 3 means no data
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
      i = i + 1;
    }
    Tachometer_Get(&LeftTach2[j], &LeftDir2, &LeftSteps2, &RightTach2[j], &RightDir2, &RightSteps2);
    Controller_Right();
    //Update Position
    UpdatePosition();
    //Get Readings & Convert
    xPos = Odometry_GetX()*.0001;
    yPos = Odometry_GetY()*.0001;
    headAngle = Odometry_GetAngle()*360/16384;

    j = j + 1;

    if(j >= TACHBUFF)
    {

      //This section of the code checks the wheel state every second (10*100ms)
      j = 0;

      //Sense state of wheels (in RPM) and take the average of the last n values
      // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
      ActualL = 2000000/avg(LeftTach2, TACHBUFF);
      ActualR = 2000000/avg(RightTach2, TACHBUFF);

      /*
      //Calculate Error signals
      Error_L = DesiredL - ActualL;
      Error_R = DesiredR - ActualR;

       DL = Kd*(Error_L-Error_L_prior)*1024;
       DR = Kd*(Error_R-Error_R_prior)*1024;

       //PID Control Law
       //Initially, only the I-term is implemented, you must add the P and D terms.
       UL = UL + Kp*Error_L + DL + (Ki*Error_L/1024);  // adjust left motor
       UR = UR + Kp*Error_R + DR + (Ki*Error_R/1024);  // adjust right motor

       PWM_RightMotor(UR);
       PWM_LeftMotor(UL);
    */
      printf("\nt = %d ms\n\r",t);
      //printf("%5d Target RPM(Left)  %5d Target RPM(Right)\n\r",DesiredL,DesiredR);
      printf("X-Position =%5d cm | Y-Position =%5d cm\n\r",xPos,yPos);
      printf("Head Angle =%5d deg\n\r",headAngle);
      printf("%5d Actual RPM(Left)  %5d Actual RPM(Right)\n\r",ActualL,ActualR);
      /*
      printf("%5d Error  RPM(Left)  %5d Error  RPM(Right)\n\r",Error_L,Error_R);
      Error_L_prior = Error_L;
      Error_R_prior = Error_R;*/

    }
    //Clock_Delay1ms(100); // delay ~0.1 sec at 48 MHz
    t = t + 100; // keep track of timer

    if(i >= 100){
      i = 0;
    }
    WaitForInterrupt();
  }
}
