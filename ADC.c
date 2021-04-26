// ADC.c
// Runs on TM4C123
// Provide functions that initialize ADC0
// Last Modified: 1/16/2021
// Student names: change this to your names or look very silly
// Last modification date: change this to the last modification date or look very silly

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

// ADC initialization function 
// Input: none
// Output: none
// measures from PD2, analog channel 5
void ADC_Init(void){ 
	uint32_t delay; 
	SYSCTL_RCGCGPIO_R |= 0x08;  // activate Port D
	while((SYSCTL_PRGPIO_R & 0x08) == 0){}; 
	GPIO_PORTD_DIR_R &= ~0x04;	// make PD2 input 
	GPIO_PORTD_AFSEL_R |= 0x04; // enable aternate fnc for PD2
	GPIO_PORTD_DEN_R &= ~0x04;  // disable digital I/O for PD2
	GPIO_PORTD_AMSEL_R |= 0x04; // enable analg fnc for PD2
	SYSCTL_RCGCADC_R |= 0x01; // activate ADC0
	delay = SYSCTL_RCGCADC_R; // extra time 2 stabilize
	delay = SYSCTL_RCGCADC_R; 
	delay = SYSCTL_RCGCADC_R; 
	delay = SYSCTL_RCGCADC_R; 
	delay = SYSCTL_RCGCADC_R; 	
	delay = SYSCTL_RCGCADC_R; 	
	delay = SYSCTL_RCGCADC_R; 	
	delay = SYSCTL_RCGCADC_R; 			
	ADC0_PC_R = 0x01; 	// congfigure for 125K
	ADC0_SSPRI_R = 0x0123; // seq3 = highest priority
	ADC0_ACTSS_R &= ~0x0008; // disable sample sequencer 3
	ADC0_EMUX_R &= ~0xF000; // seq3 is software trigger 
	ADC0_SSMUX3_R = (ADC0_SSMUX3_R & 0xFFFFFFF0)+5; // Ain5 
	ADC0_SSCTL3_R = 0x0006; // set IE0 and END0 (no TS0 and D0)
	ADC0_IM_R &= ~0x0008; // disable SS3 interrupts
	ADC0_ACTSS_R |= 0x0008; // enable sample sequencer 3

}

//------------ADC_In------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion
// measures from PD2, analog channel 5
uint32_t ADC_In(void){  
	uint32_t data; 
	ADC0_PSSI_R = 0x0008; 
	while((ADC0_RIS_R & 0x08) == 0){};
	data = ADC0_SSFIFO3_R & 0xFFF; 
	ADC0_ISC_R = 0x0008;
	return data; 
}


