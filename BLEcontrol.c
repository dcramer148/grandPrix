/*
 * BLEcontrol.c
 *
 *  Created on: Apr 13, 2023
 *      Author: David
 */


#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/UART0.h"
#include "Bump.h"
#include "motorDriver.h"

int cont;

void PORT4_IRQHandler(void)
{
    //Get which bumper was pressed
    uint8_t input = readP4();

    PWM_LeftMotor(0);
    PWM_RightMotor(0);
    //currentState = Stop;

    //Reset the flag
    P4->IFG &= input;

    cont = 0;
}


int main()
{
    Clock_Init48MHz();
    UART0_Init();
    motorPWMInit(1000, 0, 0);
    //SysTick_Init(48000,2);
    initP4();
    cont = 1;

    char input = 'a';
    while(cont)
    {
        input = UART0_InChar();
        //hex 66
        if(input == 'f')
        {
            //drive forward
            PWM_LeftMotor(400);
            PWM_RightMotor(400);
        }
        //hex 62
        else if(input == 'b')
        {
            //drive backwards
            PWM_LeftMotorBackwards(400);
            PWM_RightMotorBackwards(400);
        }
        //hex 73
        else if(input == 's')
        {
            //stop robot
            PWM_LeftMotor(0);
            PWM_RightMotor(0);
        }
        else
        {
            //do nothing
        }



    }

    return 0;
}
