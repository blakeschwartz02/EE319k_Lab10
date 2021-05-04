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
#include <stdlib.h>
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
#include "Timer2A.h"


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

//********************************************************************************

struct sprite{
	int32_t x; 
	int32_t y; 
	const unsigned char *image;
	int32_t vx, vy; 
};
typedef struct sprite sprite_t; 


sprite_t PSpotSide;    // 18 x 13

void PSpotSide_Draw(void){
	PSpotSide.x = 110; 
	PSpotSide.y = 15; 
	PSpotSide.image = ParkingSpotSide; 
	PSpotSide.vx = 0;
	PSpotSide.vy = 0; 
	SSD1306_ClearBuffer();
	SSD1306_DrawBMP(PSpotSide.x, PSpotSide.y, PSpotSide.image, 0, SSD1306_WHITE); 
	SSD1306_OutBuffer(); 
}

//------------------------------------------
sprite_t PLotSide;    // 18 x 64

uint8_t PLot_NTD = 0; 

void PLotSide_Draw(void){
//	PLot_NTD = 1; 
	PLotSide.x = 110; 
	PLotSide.y = 63; 
	PLotSide.image = ParkingLotSide; 
	PLotSide.vx = 0;
	PLotSide.vy = 0; 
//	SSD1306_ClearBuffer();
	SSD1306_DrawBMP(PLotSide.x, PLotSide.y, PLotSide.image, 0, SSD1306_WHITE); 
//	SSD1306_OutBuffer(); 
}

//------------------------------------------

sprite_t plottop;    // 18 x 31

void plot1top_Draw(void){ 
	plottop.x = 110; 
	plottop.y = 31; 
	plottop.image = plot1top; 
	plottop.vx = 0;
	plottop.vy = 0; 
	SSD1306_DrawBMP(plottop.x, plottop.y, plottop.image, 0, SSD1306_INVERSE); 
}

//------------------------------------------

sprite_t plotbottom;    // 18 x 19

void plot1bottom_Draw(void){
	plotbottom.x = 110; 
	plotbottom.y = 64; 
	plotbottom.image = plot1bottom; 
	plotbottom.vx = 0;
	plotbottom.vy = 0; 
	SSD1306_DrawBMP(plotbottom.x, plotbottom.y, plotbottom.image, 0, SSD1306_INVERSE); 
}

//------------------------------------------

sprite_t P_symbol;

	// 8 x 6
void p_symbolInit(void){
	P_symbol.x = 115; 
	P_symbol.y = 41;
	P_symbol.image = p_East; 
	P_symbol.vx = 0;
	P_symbol.vy = 0; 	
}

void p_symbol1_Draw(void){	
/*
	P_symbol.x = 115; 
	P_symbol.y = 41;
	P_symbol.image = p_East; 
	P_symbol.vx = 0;
	P_symbol.vy = 0; 
*/
	SSD1306_DrawBMP(P_symbol.x, P_symbol.y, P_symbol.image, 0, SSD1306_INVERSE); 
}

// center: (115+8)/2, (41+6)/2  = (61.5, 23.5)


//------------------------------------------

sprite_t Person;     // 12 x 14 

void PersonInit(void){   
	Person.x = 83;
	Person.y = 14; 
	Person.image = person; 
	Person.vx = 0; 
	Person.vy = 0;
}

uint8_t DirFlag = 1; 

void PersonMove(void){
	NeedToDraw1 = 1; 
	if(DirFlag == 1){
		if(Person.y == 61){   // 1 step before bottom
			DirFlag = 0; 
		}
		Person.y += 1;
	}
	else if(DirFlag == 0){
		if(Person.y == 15){  // 1 step before top 
			DirFlag = 1; 
		}
		Person.y -= 1; 
	}
}

void PersonDraw(void){
	SSD1306_ClearBuffer();
	Person.image = person;
	SSD1306_DrawBMP(Person.x, Person.y, Person.image, 0, SSD1306_WHITE); 
}

//------------------------------------------

sprite_t Tire;     // 12 x 11 

void TireInit(void){   
	Tire.x = 33;
	Tire.y = 61; 
	Tire.image = tire; 
	Tire.vx = 0; 
	Tire.vy = 0;
}

uint8_t TireDirFlag = 0; 

void TireMove(void){
	Tire_NTD = 1; 
	if(TireDirFlag == 1){
		if(Tire.y == 12){   // 1 step before bottom
			TireDirFlag = 0; 
		}
		Tire.y -= 1;
	}
	else if(TireDirFlag == 0){
		if(Tire.y == 61){  // 1 step before top 
			TireDirFlag = 1; 
		}
		Tire.y += 1; 
	}
}

void TireDraw(void){
//	SSD1306_ClearBuffer();
	Tire.image = tire;
	SSD1306_DrawBMP(Tire.x, Tire.y, Tire.image, 0, SSD1306_WHITE); 
}

//------------------------------------------

sprite_t Handicap;     //  10 x 12

void handicapInit(void){   
	Handicap.x = 1;
	Handicap.y = 15; 
	Handicap.image = handicapSmall; 
	Handicap.vx = 0; 
	Handicap.vy = 0;
}

uint8_t HCapDirFlag = 1; 

void handicapMove(void){
//	Tire_NTD = 1; 
	if(HCapDirFlag == 1){
		if(Handicap.x == 80){   // 1 step before bottom
			HCapDirFlag = 0; 
		}
		Handicap.x += 1;
	}
	else if(HCapDirFlag == 0){
		if(Handicap.x == 1){  // 1 step before top 
			HCapDirFlag = 1; 
		}
		Handicap.x -= 1; 
	}
}

void handicapDraw(void){
	SSD1306_ClearBuffer();
	Handicap.image = handicapSmall; 
	SSD1306_DrawBMP(Handicap.x, Handicap.y, Handicap.image, 0, SSD1306_WHITE); 
}
//------------------------------------------

sprite_t Ambulance;       // 20 x 15

void AmbulanceInit(void){ 
	Ambulance.x = 0; 
	Ambulance.y = 37; 
	Ambulance.image = AmbulanceSide; 
	Ambulance.vx = 2; 
	Ambulance.vy = 0; 	
}

uint8_t CrashFlag = 0; 

void AmbulanceMove(void){
//	CrashFlag = 0; 
	Ambulance.x += Ambulance.vx; 
}

void AmbulanceDraw(void){
	SSD1306_ClearBuffer();
	SSD1306_DrawBMP(Ambulance.x, Ambulance.y, Ambulance.image, 0, SSD1306_WHITE); 
	SSD1306_OutBuffer(); 	
}
//------------------------------------------

sprite_t boom; 

void BoomInit(void){ // 126 x 63 
	boom.x = 0;
	boom.y = 63;
	boom.image = Boom2; 
	boom.vx = 0;
	boom.vy = 0; 
}

void BoomDraw(void){
	SSD1306_ClearBuffer();
	SSD1306_DrawBMP(boom.x, boom.y, boom.image, 0, SSD1306_WHITE); 
}


//------------------------------------------
uint32_t Convert(uint32_t input){
// from lab 8
   return (156*input)/4096+27;
}

const unsigned char *CarDirection[8] = {CarN, CarNE, CarE, CarSE, CarS, CarSW, CarW, CarNW};

sprite_t Car[1]; 

int i = 0;

	
void CarInit(void){  
		Car[i].x = 1; 
	  Car[i].y = 42; 
	  CarFlag = 2;
		Car[i].image = CarDirection[CarFlag]; 
	  Car[i].vx = 0;
	  Car[i].vy = 0;
}

uint8_t CarStop; 

void CarMove(void){
	NeedToDraw = 1; 
	CarStop = 0; 
	if((Position > 0) && (Position <= 60)){  // fast neg velocity 
		Car[0].vx = -2; 
		Car[0].vy = -2; 
	}
	else if((Position > 60) && (Position <= 70)){ // slower neg velocity 
		Car[0].vx = -1; 
		Car[0].vy = -1;		
	}
	else if((Position > 70) && (Position <= 130)){  // stop 
		Car[0].vx = 0; 
		Car[0].vy = 0;	
		CarStop = 1; 		
	}	
	else if((Position > 130) && (Position <= 145)){ // slower pos velocity 
		Car[0].vx = 1; 
		Car[0].vy = 1;		
	}		
	else if((Position > 145) && (Position <= 190)){ // faster pos velocity 
		Car[0].vx = 2; 
		Car[0].vy = 2;		
	}	

	if(CarFlag == 0){  // N
		Car[0].y -= Car[0].vy; 
	}
	else if(CarFlag == 1){  // NE
		Car[0].x += Car[0].vx; 
		Car[0].y -= Car[0].vy; 
	}	
	else if(CarFlag == 2){ // E
		Car[0].x += Car[0].vx; 
	}		
	else if(CarFlag == 3){ // SE
		Car[0].x += Car[0].vx; 
		Car[0].y += Car[0].vy; 
	}		
	else if(CarFlag == 4){  // S
		Car[0].y += Car[0].vy; 
	}	
	else if(CarFlag == 5){ // SW
		Car[0].x -= Car[0].vx; 
		Car[0].y += Car[0].vy; 
	}	
	else if(CarFlag == 6){  // W
		Car[0].x -= Car[0].vx; 
	}	
	else if(CarFlag == 7){ // NW
		Car[0].x -= Car[0].vx; 
		Car[0].y -= Car[0].vy; 
	}	
}

void CarDraw(void){
	SSD1306_ClearBuffer();
	Car[0].image = CarDirection[CarFlag]; 
	SSD1306_DrawBMP(Car[0].x, Car[0].y, Car[0].image, 0, SSD1306_WHITE); 
}

void ParkingLot(void){
		plot1top_Draw();
	  plot1bottom_Draw();
		p_symbol1_Draw();
}

double time= 1000;
double time2 = 1000;

void ParkSuccess(void){
	// success sound interrupt 
	playSound(Win); 
  SSD1306_ClearBuffer();
  SSD1306_OutClear(); 
  SSD1306_SetCursor(3, 2);
	if(lang == 0){
		SSD1306_OutString("LEVEL 1 COMPLETE\n");	
		SSD1306_SetCursor(4, 4);
		SSD1306_OutString("Score:");	
	}
	else{
		SSD1306_OutString("LIVELLO 1 COMPLETO\n");	
		SSD1306_SetCursor(4, 4);
		SSD1306_OutString("PUNTO:");
	}
	SSD1306_OutUDec(time); 
  Delay100ms(25);	
}

void ParkSuccess2(void){
	// success sound interrupt 
	playSound(Win); 
  SSD1306_ClearBuffer();
  SSD1306_OutClear(); 
  SSD1306_SetCursor(3, 2);
	if(lang == 0){
		SSD1306_OutString("LEVEL 2 COMPLETE\n");	
		SSD1306_SetCursor(4, 4);
		SSD1306_OutString("Score:");	
	}
	else{
		SSD1306_OutString("LIVELLO 2 COMPLETO\n");	
		SSD1306_SetCursor(4, 4);
		SSD1306_OutString("PUNTO:");
	}
	SSD1306_OutUDec(time + time2); 
  Delay100ms(25);	
}

void Delay100ms(uint32_t count); // time delay in 0.1 seconds
uint32_t Position; 
uint32_t Data;

void ChooseLang(void){
	SSD1306_SetCursor(1,2);
	SSD1306_OutString("Press right button");
	SSD1306_SetCursor(2,3);
	SSD1306_OutString("for English");
	SSD1306_SetCursor(1,5);
	SSD1306_OutString("Premere il pulsante");
	SSD1306_SetCursor(2,6);
	SSD1306_OutString("sinistro per");
		SSD1306_SetCursor(3,7);
	SSD1306_OutString("l'italiano");
	//while(((GPIO_PORTF_DATA_R & 0x01) == 0x01) && ((GPIO_PORTF_DATA_R & 0x010) == 0x10)){
	//};
	while(done != 1){
	};
	SSD1306_OutClear();
}

int main(void){
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
//	Sound_Init(); 
//	playSound(BoomSound); 
/*	
  SSD1306_ClearBuffer();
  SSD1306_DrawBMP(2, 62, SpaceInvadersMarquee, 0, SSD1306_WHITE);

  SSD1306_OutBuffer();
*/
  EnableInterrupts();
 // Delay100ms(2);


  SSD1306_ClearBuffer();
	playSound(Accel); 
	SSD1306_DrawBMP(2, 62, Title, 0, SSD1306_WHITE);
	SSD1306_OutBuffer();
	Delay100ms(20);
	SSD1306_ClearBuffer();


	SSD1306_OutClear();
	ChooseLang();


	SSD1306_DrawBMP(80, 60, p, 0, SSD1306_WHITE);

//  SSD1306_OutBuffer();
 // Delay100ms(300);
 
	CarInit();
	PersonInit();


	
	p_symbolInit();
	
	int32_t P_cx = P_symbol.x + 8/2; 
	int32_t P_cy = P_symbol.y - 6/2; 	

	uint8_t success = 0; 
	uint8_t success2 = 0;
	
	while(success == 0){
		
 //   Delay100ms(10);
 //   SSD1306_SetCursor(0,0);
 //   SSD1306_OutUDec(time);
    time-= 0.01;
    PF1 ^= 0x02;
		
		if(CrashFlag == 1){
			time = 1000;
			CarInit();
			CarDraw();
			CrashFlag = 0; 
		}
		
		ParkingLot();
		
		PersonMove();
		if(NeedToDraw1 == 1){
			PersonDraw();
			NeedToDraw1 = 0; 		
			ParkingLot();
		}
		
		Data = ADC_In(); 
		Position = Convert(Data);
		if(NeedToDraw == 1){
			CarDraw(); 
			NeedToDraw = 0; 
			ParkingLot();
		}
		
		
		SSD1306_OutBuffer();

		uint8_t CarXinit = Car[0].x + 14; 
	uint8_t PerXinit = Person.x + 12;
	uint8_t CarYinit = Car[0].y - 10;
	uint8_t PerYinit = Person.y - 14;	
	uint8_t Xcheck = abs(Car[0].x - Person.x); 
	uint8_t Ycheck = abs(Car[0].y - Person.y); 
		
	if((Xcheck <= 12) && (Ycheck <= 14)){
		for(uint8_t CarX = Car[0].x; CarX <= CarXinit; CarX++){
			for(uint8_t PerX = Person.x; PerX <= PerXinit; PerX++){
				if(CarX == PerX){
					for(uint8_t CarY = Car[0].y; CarY >= CarYinit; CarY--){
						for(uint8_t PerY = Person.y; PerY >= PerYinit; PerY--){
							if(CarY == PerY){
								CrashFlag = 1;
							}
						}
					}
				}
			}
		}
	}	
	
		PersonMove();
		if(NeedToDraw1 == 1){
			PersonDraw();
			NeedToDraw1 = 0; 		
			ParkingLot();
		}
		
		Data = ADC_In(); 
		Position = Convert(Data);
		if(NeedToDraw == 1){
			CarDraw(); 
			NeedToDraw = 0; 
			ParkingLot();
		}

		SSD1306_OutBuffer();

		
		if(((Car[0].x + 14) >= 127) || (Car[0].x <= 0) || ((Car[0].y - 9) <= 0) || (Car[0].y >= 64)){ // hits wall 
			CrashFlag = 1; 
		}	
		
		int32_t Car_cx = Car[0].x + 14/2; 
		int32_t Car_cy = Car[0].y - 10/2;		
		
		int32_t cx = P_cx - Car_cx;
		int32_t cy = P_cy - Car_cy;

//		if(((P_cx >> 6)==(Car_cx >> 6)) && ((P_cy >> 5)==(Car_cy >> 5))){
//		if((abs(P_cx - Car_cx) <= 10) && (abs(P_cy - Car_cy) <= 10)){
		if((abs(cx) <= 2) && (abs(cy) <= 2) && (CarStop == 1)){
				success = 1; 
				ParkSuccess();
			CrashFlag = 0; 
		}
	
		if(CrashFlag == 1){
//			playSound(BoomSound);
//			Clock_Delay1ms(1000); 
			SSD1306_OutClear();
//			BoomInit();
 //			Timer0_Init(&wait, 80000000); 

//			BoomDraw();
//			SSD1306_OutBuffer(); 	
	    AmbulanceInit(); 
		playSound(Alarm);
			while(Ambulance.x != 120){
				AmbulanceMove(); 
				AmbulanceDraw(); 
				// siren sound interrupt 
			}
		  SSD1306_ClearBuffer();
			SSD1306_OutClear(); 
			SSD1306_SetCursor(3, 2);
			if(lang == 0){
				SSD1306_OutString("LEVEL FAILED\n");
			}
			else if(lang == 1){
				SSD1306_OutString("LIVELLO FALLITO\n");
			}
			Delay100ms(10);	
		}
				
	}
	
SSD1306_ClearBuffer();
	SSD1306_OutClear();
	SSD1306_SetCursor(4,3);
	if(lang == 0){
		SSD1306_OutString("LEVEL 2");
	}
	else{
		SSD1306_OutString("LIVELLO 2");
	}
	Delay100ms(20);
	SSD1306_OutClear();
	
	CarInit();
	Car[i].y -= 20;
	PersonInit();
	TireInit();
	
	//Level TWO ------------------------------------------------
	while(success2 == 0){
		
 //   Delay100ms(10);
 //   SSD1306_SetCursor(0,0);
 //   SSD1306_OutUDec(time);
    time2-= 1;
    PF1 ^= 0x02;
		
		if(CrashFlag == 1){
			time2 = 1000;
			CarInit();
			Car[i].y -= 20;
			CarDraw();
			CrashFlag = 0; 
		}
		
		ParkingLot();
		
		SSD1306_OutBuffer();

		PersonMove();
		if(NeedToDraw1 == 1){
			PersonDraw();
			NeedToDraw1 = 0; 		
			ParkingLot();
		}
		
		TireMove();
		if(Tire_NTD == 1){
			TireDraw();
			Tire_NTD = 0; 
			ParkingLot();
		}
		
		SSD1306_OutBuffer();
		
		Data = ADC_In(); 
		Position = Convert(Data);
		if(NeedToDraw == 1){
			CarDraw(); 
			NeedToDraw = 0; 
			ParkingLot();
		}
		
		
		SSD1306_OutBuffer();

	uint8_t CarXinit = Car[0].x + 14; 
	uint8_t PerXinit = Person.x + 12;
	uint8_t CarYinit = Car[0].y - 10;
	uint8_t PerYinit = Person.y - 14;	
	uint8_t Xcheck = abs(Car[0].x - Person.x); 
	uint8_t Ycheck = abs(Car[0].y - Person.y); 
		
	if(((Xcheck <= 12) && (Ycheck <= 14))){
		for(uint8_t CarX = Car[0].x; CarX <= CarXinit; CarX++){
			for(uint8_t PerX = Person.x; PerX <= PerXinit; PerX++){
				if(CarX == PerX){
					for(uint8_t CarY = Car[0].y; CarY >= CarYinit; CarY--){
						for(uint8_t PerY = Person.y; PerY >= PerYinit; PerY--){
							if(CarY == PerY){
								CrashFlag = 1;
							}
						}
					}
				}
			}
		}
	}	
	
		PersonMove();
		if(NeedToDraw1 == 1){
			PersonDraw();
			NeedToDraw1 = 0; 		
			ParkingLot();
		}

	SSD1306_OutBuffer();
		
	uint8_t TireXinit = Tire.x + 12;
	uint8_t TireYinit = Tire.y - 11;	
	uint8_t XTirecheck = abs(Car[0].x - Tire.x); 
	uint8_t YTirecheck = abs(Car[0].y - Tire.y); 
		
	if((XTirecheck <= 12) && (YTirecheck <= 11)){
		for(uint8_t CarX = Car[0].x; CarX <= CarXinit; CarX++){
			for(uint8_t TireX = Tire.x; TireX <= TireXinit; TireX++){
				if(CarX == TireX){
					for(uint8_t CarY = Car[0].y; CarY >= CarYinit; CarY--){
						for(uint8_t TireY = Tire.y; TireY >= TireYinit; TireY--){
							if(CarY == TireY){
								CrashFlag = 1;
							}
						}
					}
				}
			}
		}
	}	
	
		TireMove();
		if(Tire_NTD == 1){
			TireDraw();
			Tire_NTD = 0; 
			ParkingLot();
		}
		
		SSD1306_OutBuffer();
		
		Data = ADC_In(); 
		Position = Convert(Data);
		if(NeedToDraw == 1){
			CarDraw(); 
			NeedToDraw = 0; 
			ParkingLot();
		}

		SSD1306_OutBuffer();

		
		if(((Car[0].x + 14) >= 127) || (Car[0].x <= 0) || ((Car[0].y - 9) <= 0) || (Car[0].y >= 64)){ // hits wall 
			CrashFlag = 1; 
		}	
		
		int32_t Car_cx = Car[0].x + 14/2; 
		int32_t Car_cy = Car[0].y - 10/2;		
		
		int32_t cx = P_cx - Car_cx;
		int32_t cy = P_cy - Car_cy;

//		if(((P_cx >> 6)==(Car_cx >> 6)) && ((P_cy >> 5)==(Car_cy >> 5))){
//		if((abs(P_cx - Car_cx) <= 10) && (abs(P_cy - Car_cy) <= 10)){
		if((abs(cx) <= 2) && (abs(cy) <= 2) && (CarStop == 1)){
				success2 = 1; 
				ParkSuccess2();
			CrashFlag = 0; 
		}
	
		if(CrashFlag == 1){
//			playSound(BoomSound);
//			Clock_Delay1ms(1000); 
			SSD1306_OutClear();
//			BoomInit();
 //			Timer0_Init(&wait, 80000000); 

//			BoomDraw();
//			SSD1306_OutBuffer(); 	
	    AmbulanceInit(); 
		playSound(Alarm);
			while(Ambulance.x != 120){
				AmbulanceMove(); 
				AmbulanceDraw(); 
				// siren sound interrupt 
			}
		  SSD1306_ClearBuffer();
			SSD1306_OutClear(); 
			SSD1306_SetCursor(3, 2);
			if(lang == 0){
				SSD1306_OutString("LEVEL FAILED\n");
			}
			else if(lang == 1){
				SSD1306_OutString("LIVELLO FALLITO\n");
			}
			Delay100ms(10);	
		}
				
	}
		
	
	SSD1306_OutClear(); 
	while(1){ 
		SSD1306_SetCursor(2, 2);
		if(lang == 0){
			SSD1306_OutString("END OF GAME");
		}
		else{
			SSD1306_OutString("FINE DEL GIOCO");
		}
	}		
}


//	SSD1306_OutBuffer();
/*	
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
*/	
	
/*  
  SSD1306_DrawBMP(47, 63, PlayerShip0, 0, SSD1306_WHITE); // player ship bottom
  SSD1306_DrawBMP(53, 55, Bunker0, 0, SSD1306_WHITE);
	SSD1306_DrawBMP(0, 9, Alien10pointA, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(20,9, Alien10pointB, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(40, 9, Alien20pointA, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(60, 9, Alien20pointB, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(80, 9, Alien30pointA, 0, SSD1306_WHITE);
  SSD1306_DrawBMP(50, 19, AlienBossA, 0, SSD1306_WHITE);

}
*/

/*
		PersonMove();
		
		if(NeedToDraw1 == 1){
			PersonDraw();
			NeedToDraw1 = 0; 		
			ParkingLot();
		}
*/

