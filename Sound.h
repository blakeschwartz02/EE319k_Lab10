// Sound.h
// Runs on TM4C123 
// Prototypes for basic functions to play sounds from the
// original Space Invaders.
// Jonathan Valvano
// 1/14/21
#ifndef _Sound_h
#define _Sound_h
#include <stdint.h>

// Header files contain the prototypes for public functions 
// this file explains what the module does


typedef enum {Alarm, Win, Accel} soundEffect; 

extern uint32_t pIndex;
void playSample(void);

void playSound(soundEffect);

void Sound_Init(void);

void Sound_Play(const uint8_t *pt, uint32_t count);


#endif


