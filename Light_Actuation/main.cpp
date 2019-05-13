#include "mbed.h"

#define PRESCALE 24000 - 1

// For use with Cognition

void initPINs();
void initTIM2();
void enableInputCapture();
void startTIM2();

volatile unsigned int lightSenseTime;
unsigned int pressSenseTime;
unsigned int soundSenseTime;
unsigned int currTVal;

int main() {    
    printf("SystemCoreClock = %d Hz\n\r", SystemCoreClock);
    
    initPINs();
    initTIM2();
    enableInputCapture();
    startTIM2();
    NVIC_EnableIRQ(TIMER2_IRQn);
    printf("READY\n\r");    
}

void initPINs() {
    LPC_GPIO0->FIODIR |= (1 << 1);                  // Set P0.0 (p9) output
    LPC_GPIO1->FIODIR |= (1 << 23);                 // Set (led4) output
    
    printf("INIT PINS\n\r");
}

void initTIM2() {
    LPC_SC->PCONP |= (1 << 22);                     // Power on TIM2
    LPC_SC->PCLKSEL1 &= ~(0x3 << 13);               // Set prescaler timer to
                                                    // 1/4 of main clock
                                                    
    LPC_TIM2->CTCR = 0x0;                           // Set TIM2 to timer mode
    LPC_TIM2->PR = PRESCALE;                        // Set TIM2 prescale (1 tick per ms)
    
    printf("INIT TIM2\n\r");
}

void enableInputCapture() {
    LPC_TIM2->CCR |= (1 << 1);                      // Set CAP2.0 falling edge (light)    
    LPC_PINCON->PINSEL0 |= (1 << 9) | (1 << 8);     // Set p30 as CAP2.0    
    LPC_TIM2->CCR |= (1 << 2);                      // Enable CAP2.0 interrupt
    
    printf("ENABLE IC\n\r");
}

void startTIM2() {        
    LPC_TIM2->TCR = 0x02;                           // Reset Timer
    LPC_TIM2->TCR = 0x01;                           // Enable timer

    printf("START TIM2\n\r");
}

extern "C" void TIMER2_IRQHandler(void) __irq {
    LPC_TIM2->IR |= (1 << 1);                       // Clear interrupt flag
    LPC_TIM2->CCR &= ~(1 << 2);                     // Disable CAP2.0 interrupt 
    
    // Actuate the solenoid
    LPC_GPIO1->FIOSET |= (1 << 23);                // Turn on LED
    LPC_GPIO0->FIOSET |= (1 << 1);                 // Send voltage to p10
    printf("Signal received\n\r");
    wait(1);
    LPC_GPIO0->FIOCLR |= (1 << 1);                 // Stop voltage to p10
    LPC_GPIO1->FIOCLR |= (1 << 23);                // Turn off LED
    wait(0.75);
    
    enableInputCapture();
    startTIM2();
    
    
    // printf("Int2 \n\r");
}