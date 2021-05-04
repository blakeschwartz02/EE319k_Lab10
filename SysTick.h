

#include <stdint.h>

extern uint8_t NeedToDraw1; 

extern uint8_t Tire_NTD;

extern uint8_t handicap_NTD; 


void SysTick_Init(uint32_t period);


void SysTick_Handler(void);
