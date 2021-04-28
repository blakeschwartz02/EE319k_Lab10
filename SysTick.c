
#include <stdint.h>
#include "SSD1306.h"
#include "TExaS.h"
#include "../inc/tm4c123gh6pm.h"



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
int St = N; 
int Input; 

void SysTick_Handler(void){
//		while(((GPIO_PORTE_DATA_R | 0xFFFFFFFE) == 0xFFFFFFFE) || ((GPIO_PORTE_DATA_R | 0xFFFFFFFD) == 0xFFFFFFFD)){
//		}	
		CarFlag = FSM[St].outF;
		Input = GPIO_PORTE_DATA_R | 0xFFFFFFF8; // 2 buttons + slide pot
		St = FSM[St].next[Input]; 

}

 /* while(1){
		PB543210 = FSM[S].outC; // output
		PF321 = FSM[S].outP;
		SysTick_Wait10ms(FSM[S].wait); // wait
		Input = PE210; // input
		S = FSM[S].next[Input]; // next	
  }	
*/


