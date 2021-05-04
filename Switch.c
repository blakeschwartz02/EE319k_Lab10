// Switch.c
// This software to input from switches or buttons
// Runs on TM4C123
// Program written by: put your names here
// Date Created: 3/6/17 
// Last Modified: 1/14/21
// Lab number: 10
// Hardware connections
// TO STUDENTS "REMOVE THIS LINE AND SPECIFY YOUR HARDWARE********

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "ADC.h"
#include "Switch.h"

// Code files contain the actual implemenation for public functions
// this file also contains an private functions and private data


volatile uint8_t right, left; // semaphores 
void Switch_Init(void){
	//PortE
	SYSCTL_RCGCGPIO_R |= 0x00000010; // activate Port E clock
	right = 0; left = 0; // clear semaphores 
	while((SYSCTL_PRGPIO_R & 0x00000010) == 0){};
	GPIO_PORTE_AMSEL_R &= ~0x03; // disable analog function 
	GPIO_PORTE_PCTL_R &= ~0x000000FF; // configure as GPIO
	GPIO_PORTE_DIR_R &= ~0x03; // make PE1-0 inputs
	GPIO_PORTE_AFSEL_R &= ~ 0x03; 
	GPIO_PORTE_DEN_R |= 0x03; 
	GPIO_PORTE_PDR_R |= 0x03; // pull-down 
	GPIO_PORTE_IS_R &= ~0x03; // PE1-0 edge sensitive
	GPIO_PORTE_IBE_R &= ~0x03; // PE1-0 not both edges 
	GPIO_PORTE_IEV_R |= 0x03; // PE1-0 rising edge
	GPIO_PORTE_ICR_R = 0x03; // clear flag1-0
	GPIO_PORTE_IM_R |= 0x03; // arm interrupt 
	NVIC_PRI1_R = (NVIC_PRI1_R & 0xFFFFFF00) | 0x00000040; // priority 2
	//PortF
		/*
	SYSCTL_RCGCGPIO_R |= 0x00000020; // activate Port F clock
	while((SYSCTL_PRGPIO_R & 0x00000020) == 0){};
	GPIO_PORTF_LOCK_R = 0x4C4F434B;
	GPIO_PORTF_CR_R = 0x1F;
	GPIO_PORTF_AMSEL_R &= ~0x11; // disable analog function 
	GPIO_PORTF_PCTL_R &= ~0x000000FF; // ******* configure as GPIO
	GPIO_PORTF_DIR_R &= ~0x11; // make PF4-0 inputs
	GPIO_PORTF_AFSEL_R &= ~ 0x11; 
	GPIO_PORTF_DEN_R |= 0x11; 
	GPIO_PORTF_PUR_R |= 0x11; // pull-up
	GPIO_PORTF_IS_R &= ~0x11; // PF4,0 edge sensitive
	GPIO_PORTF_IBE_R &= ~0x11; // PF4,0 not both edges 
	GPIO_PORTF_IEV_R |= 0x11; // PF4,0 rising edge
	GPIO_PORTF_ICR_R = 0x11; // clear flag4,0
	GPIO_PORTF_IM_R |= 0x11; // arm interrupt 
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00200000; // priority 1
	*/
	NVIC_EN0_R = 0x00000010; // enable interrupt 4 and 5 in NVIC (Port E, Port F) 
	EnableInterrupts(); 
}

struct State{
//	const unsigned char *out; // output for Sprite
	uint32_t outF; // output for car flag
//	uint32_t outSP; // output for slide pot 
	uint32_t next[8]; 
};

typedef const struct State State_t; 

#define N 0 
#define NE 1 
#define E 2 
#define SE 3 
#define S 4 
#define SW 5 
#define W 6 
#define NW 7 

State_t FSM[8] = {
//outF 111 110 101 100 011 010 001 000 
	{0, {N, NW, NE, N, N, NE, NW, N}},
	{1, {NE, N, E, NE, NE, E, N, NE}},
	{2, {E, NE, SE, E, E, SE, NE, E}},
	{3, {SE, E, S, SE, SE, S, E, SE}},
	{4, {S, SE, SW, S, S, SW, SE, S}},
	{5, {SW, S, W, SW, SW, W, S, SW}},
	{6, {W, SW, NW, W, W, NW, SW, W}},
	{7, {NW, W, N, NW, NW, N, W, NW}}
};



int CarFlag; 
int St = E; 
int Input; 
int lang;
int done = 0;
 
uint8_t NeedToDraw = 0; 

void GPIOPortE_Handler(void){

	GPIO_PORTE_ICR_R = 0x03; // ack PE1-0
	
	if(done == 0){
		if((GPIO_PORTE_DATA_R & 0x00000003) == 0x01){
			lang = 0;
		}
		else if((GPIO_PORTE_DATA_R & 0x00000003) == 0x02){
			lang = 1;
		}
		done = 1;
	}
	
	if(Position <= 90){
		Input = 0x00000000; // bit 2 is 0 for neg 
	}
	else{ 
		Input = 0x00000004; // bit 2 is 1 for pos 
	}

	CarFlag = FSM[St].outF;
	NeedToDraw = 1; 
	Input += (GPIO_PORTE_DATA_R & 0x00000003); // 2 buttons + slide pot
	St = FSM[St].next[Input]; 
}

/*
void GPIOPortF_Handler(void){
	
	GPIO_PORTF_ICR_R = 0x11;
	
	if((GPIO_PORTF_DATA_R & 0x00000011) == 0x01){
		lang = 0;
		done = 1;
	}
  else if((GPIO_PORTF_DATA_R & 0x00000011) == 0x10){
		lang = 1;
		done = 1;
	}
}

*/