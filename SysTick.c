
#include <stdint.h>
#include "SSD1306.h"
#include "TExaS.h"
#include "../inc/tm4c123gh6pm.h"
#include "Switch.h"
#include "ADC.h"

extern void CarMove(void); 


// **************SysTick_Init*********************
// Initialize Systick periodic interrupts
// Input: interrupt period
//        Units of period are 12.5ns
//        Maximum is 2^24-1
//        Minimum is determined by length of ISR
// Output: none
void SysTick_Init(uint32_t period){
    // write this
	NVIC_ST_CTRL_R = 0;    // disable SysTick
	NVIC_ST_RELOAD_R = period - 1;   // max reload value
	NVIC_ST_CURRENT_R = 0; // write to CURRENT
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x20000000; // priority
	NVIC_ST_CTRL_R = 7; // ENABLE, INTEN, CLK SRC	
}

uint8_t NeedToDraw1 = 0; 
uint8_t Tire_NTD = 0; 
uint8_t handicap_NTD = 0; 


void SysTick_Handler(void){
	CarMove();
	}




