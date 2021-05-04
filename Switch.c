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
	NVIC_EN0_R = 0x00000010; // enable interrupt 4 and 5 in NVIC (Port E) 
	EnableInterrupts(); 
}

struct State{
	uint32_t outF; // output for car flag
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

