#include "mbed.h"

#define PRESCALE 24000 - 1

void initPINs();
void initTIM2();
void enableInputCapture();
void startTIM2();

unsigned int lightSenseTime;
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
    NVIC_EnableIRQ(EINT3_IRQn);
    printf("READY\n\r");    
}

void initPINs() {
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

void initFILE() {
    FILE *fp = fopen("/local/Data.txt", "w");
    if (fp == NULL) {
        printf("File error\r\n");
    } else {        
        fprintf(fp, "SND-LGT,SND-PRS");
        fclose(fp);
    }
}  

void enableInputCapture() {
    LPC_TIM2->CCR |= (1 << 4);                      // Set CAP2.1 falling edge (press)
    
    LPC_PINCON->PINSEL0 |= (1 << 11) | (1 << 10);   // Set p29 as CAP2.1
        
    LPC_TIM2->CCR |= (1 << 5);                      // Enable CAP2.1 interrupt
    
    printf("ENABLE IC\n\r");
}

void startTIM2() {        
    LPC_TIM2->TCR = 0x02;                           // Reset Timer
    LPC_TIM2->TCR = 0x01;                           // Enable timer

    printf("START TIM2\n\r");
}

extern "C" void EINT3_IRQHandler(void) __irq {
    LPC_GPIOINT->IO0IntClr |= (1 << 25);            // Clear interrupt flag
    LPC_GPIOINT->IO0IntEnR &= ~(1 << 25);           // Disable GPIO rising interrupt
    
    LPC_TIM2->TCR = 0x00;                           // Disable TIM2
    
    LPC_GPIO0->FIOCLR |= (1 << 0);                  // Stop voltage to p20 (solenoid)
    
    LPC_GPIO1->FIOCLR |= (1 << 23); 
    
    printf("DISARMED\n\r");
    
    lightSenseTime = LPC_TIM2->CR0;
    pressSenseTime = LPC_TIM2->CR1;
    soundSenseTime = LPC_TIM2->TC;
    
    
    printf("LGT = %d\n\r", lightSenseTime);
    printf("PRS = %d\n\r", pressSenseTime);
    printf("SND = %d\n\r", soundSenseTime);
    printf("SND - LGT = %d\n\r", soundSenseTime - lightSenseTime);
    printf("SND - PRS = %d\n\r", soundSenseTime - pressSenseTime);    
    
    
//    FILE *fp = fopen("/local/Variables.txt", "w");
//    if (fp == NULL) {
//        printf("File error\r\n");
//    } else {        
//        fprintf(fp, "%d,%d", soundSenseTime - lightSenseTime, soundSenseTime - pressSenseTime);
//        fclose(fp);
//    }
    
    wait(2.5);
    printf("REARMED\n\r");
    // enableInputCapture();
    // startTIM2();
    
    LPC_TIM2->TCR = 0x02;
    
    LPC_TIM2->CCR |= (1 << 1);                      // Set CAP2.0 falling edge (light)
    LPC_TIM2->CCR |= (1 << 4);                      // Set CAP2.1 falling edge (press)
    
    LPC_PINCON->PINSEL0 |= (1 << 9) | (1 << 8);     // Set p30 as CAP2.0
    LPC_PINCON->PINSEL0 |= (1 << 11) | (1 << 10);   // Set p29 as CAP2.1
    
    LPC_GPIOINT->IO0IntEnR |= (1 << 25);            // Set GPIO rising interrupt
    
    LPC_TIM2->CCR |= (1 << 2);                      // Enable CAP2.0 interrupt
    
    printf("ENABLE IC\n\r");
    
    LPC_TIM2->TCR = 0x02;                           // Reset Timer
    LPC_TIM2->TCR = 0x01;                           // Enable timer

    printf("START TIM2\n\r");
}

extern "C" void TIMER2_IRQHandler(void) __irq {
    LPC_TIM2->IR |= (1 << 1);                       // Clear interrupt flag
    //LPC_TIM2->CCR &= ~(1 << 5);                     // Disable CAP2.1 interrupt 
    
    // printf("CR1 = %d\n", LPC_TIM2->CR0);
    
    // Actuate the solenoid
    LPC_GPIO1->FIOSET |= (1 << 23);                // Turn on LED
    LPC_GPIO0->FIOSET |= (1 << 0);                 // Send voltage to p9
    LPC_GPIO1->FIOCLR |= (1 << 23);                // Turn off LED
    
     printf("Int2 \n\r");
}