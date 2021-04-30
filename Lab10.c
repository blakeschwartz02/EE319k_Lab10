// Lab10.c
// Runs on TM4C123
// Jonathan Valvano and Daniel Valvano
// This is a starter project for the EE319K Lab 10

// Last Modified: 1/16/2021 
// http://www.spaceinvaders.de/
// sounds at http://www.classicgaming.cc/classics/spaceinvaders/sounds.php
// http://www.classicgaming.cc/classics/spaceinvaders/playguide.php
/* 
 Copyright 2021 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
// ******* Possible Hardware I/O connections*******************
// Slide pot pin 1 connected to ground
// Slide pot pin 2 connected to PD2/AIN5
// Slide pot pin 3 connected to +3.3V 
// fire button connected to PE0
// special weapon fire button connected to PE1
// 8*R resistor DAC bit 0 on PB0 (least significant bit)
// 4*R resistor DAC bit 1 on PB1
// 2*R resistor DAC bit 2 on PB2
// 1*R resistor DAC bit 3 on PB3 (most significant bit)
// LED on PB4
// LED on PB5

// VCC   3.3V power to OLED
// GND   ground
// SCL   PD0 I2C clock (add 1.5k resistor from SCL to 3.3V)
// SDA   PD1 I2C data

//************WARNING***********
// The LaunchPad has PB7 connected to PD1, PB6 connected to PD0
// Option 1) do not use PB7 and PB6
// Option 2) remove 0-ohm resistors R9 R10 on LaunchPad
//******************************

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "SSD1306.h"
#include "Print.h"
#include "Random.h"
#include "ADC.h"
#include "Images.h"
#include "Sound.h"
#include "Timer0.h"
#include "Timer1.h"
#include "TExaS.h"
#include "Switch.h"
#include "SysTick.h"


//********************************************************************************
// debuging profile, pick up to 7 unused bits and send to Logic Analyzer
#define PB54                  (*((volatile uint32_t *)0x400050C0)) // bits 5-4
#define PF321                 (*((volatile uint32_t *)0x40025038)) // bits 3-1
// use for debugging profile
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define PB5       (*((volatile uint32_t *)0x40005080)) 
#define PB4       (*((volatile uint32_t *)0x40005040)) 
// TExaSdisplay logic analyzer shows 7 bits 0,PB5,PB4,PF3,PF2,PF1,0 
// edit this to output which pins you use for profiling
// you can output up to 7 pins
void LogicAnalyzerTask(void){
  UART0_DR_R = 0x80|PF321|PB54; // sends at 10kHz
}
void ScopeTask(void){  // called 10k/sec
  UART0_DR_R = (ADC1_SSFIFO3_R>>4); // send ADC to TExaSdisplay
}
// edit this to initialize which pins you use for profiling
void Profile_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x22;      // activate port B,F
  while((SYSCTL_PRGPIO_R&0x20) != 0x20){};
  GPIO_PORTF_DIR_R |=  0x0E;   // output on PF3,2,1 
  GPIO_PORTF_DEN_R |=  0x0E;   // enable digital I/O on PF3,2,1
  GPIO_PORTB_DIR_R |=  0x30;   // output on PB4 PB5
  GPIO_PORTB_DEN_R |=  0x30;   // enable on PB4 PB5  
}
//********************************************************************************

const unsigned char *CarDirection[8] = {CarN, CarNE, CarE, CarSE, CarS, CarSW, CarW, CarNW};

struct sprite{
	int32_t x; 
	int32_t y; 
	const unsigned char *image;
	int32_t vx, vy; 
};
typedef struct sprite sprite_t; 

sprite_t Car[1]; 

void CarInit(void){ int i = 0; 
		Car[i].x = 0; 
	  Car[i].y = 39; 
		Car[i].image = CarDirection[2]; 
	  Car[i].vx = 0;
	  Car[i].vy = 0;
//	  SSD1306_DrawBMP(Car[0].x, Car[0].y, Car[0].image, 0, SSD1306_WHITE); 
}
sprite_t ParkingSpot[3];

sprite_t Person; 

void PersonInit(void){  // 12 x 14  
	Person.x = 90;
	Person.y = 14; 
	Person.image = person; 
	Person.vx = 0; 
	Person.vy = 0;
}

uint32_t Convert(uint32_t input){
// from lab 8
   return (156*input)/4096+27;
}

void CarMove(void){
	NeedToDraw = 1; 
	
	if((Position > 0) && (Position <= 60)){  // fast neg velocity 
		Car[0].vx = -2; 
		Car[0].vy = -2; 
	}
	else if((Position > 60) && (Position <= 90)){ // slower neg velocity 
		Car[0].vx = -1; 
		Car[0].vy = -1;		
	}
	else if((Position > 90) && (Position <= 110)){  // stop 
		Car[0].vx = 0; 
		Car[0].vy = 0;		
	}	
	else if((Position > 110) && (Position <= 145)){ // slower pos velocity 
		Car[0].vx = 1; 
		Car[0].vy = 1;		
	}		
	else if((Position > 145) && (Position <= 190)){ // faster pos velocity 
		Car[0].vx = 3; 
		Car[0].vy = 3;		
	}	

	if(CarFlag == 0){  // N
		Car[0].y -= Car[0].vy; 
	}
	else if(CarFlag == 1){  // NE
		Car[0].x += Car[0].vx; 
		Car[0].y += Car[0].vy; 
	}	
	else if(CarFlag == 2){ // E
		Car[0].x += Car[0].vx; 
	}		
	else if(CarFlag == 3){ // SE
		Car[0].x += Car[0].vx; 
		Car[0].y -= Car[0].vy; 
	}		
	else if(CarFlag == 4){  // S
		Car[0].y += Car[0].vy; 
	}	
	else if(CarFlag == 5){ // SW
		Car[0].x -= Car[0].vx; 
		Car[0].y -= Car[0].vy; 
	}	
	else if(CarFlag == 6){  // W
		Car[0].x -= Car[0].vx; 
	}	
	else if(CarFlag == 7){ // NW
		Car[0].x -= Car[0].vx; 
		Car[0].y += Car[0].vy; 
	}	
}

void CarDraw(void){
	SSD1306_ClearBuffer();
	Car[0].image = CarDirection[CarFlag]; 
	SSD1306_DrawBMP(Car[0].x, Car[0].y, Car[0].image, 0, SSD1306_WHITE); 
	SSD1306_OutBuffer(); 
	NeedToDraw = 0; 
}





void Delay100ms(uint32_t count); // time delay in 0.1 seconds
uint32_t Position; 
uint32_t Data;

int main(void){uint32_t time=0;
  DisableInterrupts();
  // pick one of the following three lines, all three set to 80 MHz
  //PLL_Init();                   // 1) call to have no TExaS debugging
  TExaS_Init(&LogicAnalyzerTask); // 2) call to activate logic analyzer
  //TExaS_Init(&ScopeTask);       // or 3) call to activate analog scope PD2
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_OutClear();   
  Random_Init(1);
  Profile_Init(); // PB5,PB4,PF3,PF2,PF1 
	SysTick_Init(4000000); 
	ADC_Init();
	Switch_Init();
	
  SSD1306_ClearBuffer();
  SSD1306_DrawBMP(2, 62, SpaceInvadersMarquee, 0, SSD1306_WHITE);

  SSD1306_OutBuffer();
  EnableInterrupts();
  Delay100ms(2);
  SSD1306_ClearBuffer();

	//SSD1306_DrawBMP(2, 0, UpArrow, 0, SSD1306_WHITE);
/*	SSD1306_DrawBMP(2, 39, CarN, 0, SSD1306_WHITE);
	SSD1306_DrawBMP(20, 39, CarNE, 0, SSD1306_WHITE);

	SSD1306_DrawBMP(60, 39, CarSE, 0, SSD1306_WHITE);

	SSD1306_DrawBMP(2, 60, CarW, 0, SSD1306_WHITE);
	SSD1306_DrawBMP(20, 60, CarNW, 0, SSD1306_WHITE);
	SSD1306_DrawBMP(80, 60, p, 0, SSD1306_WHITE);
*/
//	SSD1306_DrawBMP(60, 30, AmbulanceE, 0, SSD1306_WHITE);
//	SSD1306_DrawBMP(80, 20, ParkingLotSide, 0, SSD1306_WHITE);

//	SSD1306_DrawBMP(90, 14, person, 0, SSD1306_WHITE);
//	SSD1306_DrawBMP(95, 39, CarS, 0, SSD1306_WHITE);
//	SSD1306_DrawBMP(100, 39, CarSW, 0, SSD1306_WHITE);
//	SSD1306_DrawBMP(60, 60, AmbulanceE_F, 0, SSD1306_WHITE);

/*  
  SSD1306_DrawBMP(47, 63, PlayerShip0, 0, SSD1306_WHITE); // player ship bottom
  SSD1306_DrawBMP(53, 55, Bunker0, 0, SSD1306_WHITE);
	SSD1306_DrawBMP(0, 9, Alien10pointA, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(20,9, Alien10pointB, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(40, 9, Alien20pointA, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(60, 9, Alien20pointB, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(80, 9, Alien30pointA, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(50, 19, AlienBossA, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(0, 39, CarE, 0, SSD1306_WHITE);
  SSD1306_OutBuffer();
  Delay100ms(300);
*/
	CarInit();
//	SSD1306_OutBuffer();
//  Delay100ms(300);
	

	while(1){
		Data = ADC_In(); 
		Position = Convert(Data);
		if(NeedToDraw == 1){
			CarDraw(); 
		}
	}
	
  Delay100ms(300);
//	SSD1306_OutBuffer();
	



  SSD1306_OutClear();  
  SSD1306_SetCursor(1, 1);
  SSD1306_OutString("GAME OVER");
  SSD1306_SetCursor(1, 2);
  SSD1306_OutString("Nice try,");
  SSD1306_SetCursor(1, 3);
  SSD1306_OutString("Earthling!");
  SSD1306_SetCursor(2, 4);
  while(1){
    Delay100ms(10);
    SSD1306_SetCursor(19,0);
    SSD1306_OutUDec2(time);
    time++;
    PF1 ^= 0x02;
  }
}

// You can't use this timer, it is here for starter code only 
// you must use interrupts to perform delays
void Delay100ms(uint32_t count){uint32_t volatile time;
  while(count>0){
    time = 727240;  // 0.1sec at 80 MHz
    while(time){
	  	time--;
    }
    count--;
  }
}
