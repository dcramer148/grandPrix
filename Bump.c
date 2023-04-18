/*
 * Bump.c
 *
 *  Created on: Feb 20, 2023
 *      Author: jsb19
 */

/*
 * 4.7 Bump5 Left
 * 4.6
 * 4.5
 * 4.3
 * 4.2
 * 4.0 Bump0 Right
 */
#include "Bump.h"
#include "msp.h"
#include "CortexM.h"


//Initialize port 4 for input and interrupts
void initP4(void)
{
    P4->SEL0 &= ~(B0 + B1 + B2 + B3 + B4 + B5);
    P4->SEL1 &= ~(B0 + B1 + B2 + B3 + B4 + B5);
    P4->DIR &= ~(B0 + B1 + B2 + B3 + B4 + B5);
    P4->REN |= (B0 + B1 + B2 + B3 + B4 + B5);
    P4->OUT |= (B0 + B1 + B2 + B3 + B4 + B5);
    P4->IES |= (B0 + B1 + B2 + B3 + B4 + B5);
    P4->IFG &= ~(B0 + B1 + B2 + B3 + B4 + B5);
    P4->IE |= (B0 + B1 + B2 + B3 + B4 + B5);

    NVIC->IP[9] = (NVIC->IP[9]&0xFF0FFFFF) | 0x00C00000;
    NVIC->ISER[1] = 0x00000040;
    EnableInterrupts();
}

//Read data from port 4
uint8_t readP4(void)
{
    return (P4->IN&(B0 + B1 + B2 + B3 + B4 + B5)) ^ 0xFF;
}

