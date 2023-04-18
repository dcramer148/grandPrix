/*
 * motorDriver.h
 *
 *  Created on: Feb 24, 2023
 *      Author: jsb19
 */

#ifndef MOTORDRIVER_H_
#define MOTORDRIVER_H_

#include <stdint.h>
#include "msp.h"
#include "CortexM.h"

void motorPWMInit(uint16_t period, uint16_t duty1, uint16_t duty2);
void PWM_LeftMotor(uint16_t duty);
void PWM_RightMotor(uint16_t duty);
void PWM_LeftMotorBackwards(uint16_t duty);
void PWM_RightMotorBackwards(uint16_t duty);


#endif /* MOTORDRIVER_H_ */
