#include <stdint.h>
#include "msp.h"

void (*TimerA1Task)(void); // user function

// ***************** TimerA1_Init ****************
// Activate Timer A1 interrupts to run user task periodically
// Inputs: task is a pointer to a user function
// period in units (24/SMCLK), 16 bits
// Outputs: none
// With SMCLK 12 MHz, period has units 2us
void TimerA1_Init(void(*task)(void), uint16_t period){
TimerA1Task = task; // user function
TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
TIMER_A1->CCR[0] = period; // TACCR0 set to period
TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK, continuous mode
TIMER_A_CTL_MC__UP | // Up mode, count to TACCR0
TIMER_A_CTL_ID__1; // Divide clock by 1
NVIC_EnableIRQ(TA1_0_IRQn); // Enable TA1_0_IRQn (Timer A1 CC0) interrupt
}

// ------------TimerA1_Stop------------
// Deactivate the interrupt running a user task periodically.
// Input: none
// Output: none
void TimerA1_Stop(void){
TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIE; // TACCR0 interrupt disabled
TIMER_A1->CTL = TIMER_A_CTL_MC__STOP; // Stop timer
NVIC_DisableIRQ(TA1_0_IRQn); // Disable TA1_0_IRQn (Timer A1 CC0) interrupt
}

void TA1_0_IRQHandler(void){
TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; // Clear TACCR0 interrupt flag
(*TimerA1Task)(); // Call user function
}
