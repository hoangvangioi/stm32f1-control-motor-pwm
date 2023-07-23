#include "delay.h"

void Delay_Init();
void Delay_1us();
void Delay_us(unsigned int _utime);
void Delay_1ms();
void Delay_ms(unsigned int _mtime);


void Delay_Init(){
	TIM_TimeBaseInitTypeDef timer_init;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	timer_init.TIM_CounterMode = TIM_CounterMode_Up;
	timer_init.TIM_Period = 65535;
	timer_init.TIM_Prescaler = 1;
	timer_init.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&timer_init);
}

/*
function: 1us
*/
void Delay_1us(){
	  // set gia tri dem cua timer
		Delay_Init();
		TIM_Cmd(TIM2, ENABLE);
		TIM_SetCounter(TIM2, 0);
		while(TIM_GetCounter(TIM2) < 36);
		TIM_Cmd(TIM2, DISABLE);
}

void Delay_us(unsigned int _utime){
	while(_utime--){
		Delay_1us();
	}
}

/*
function: 1ms
*/
void Delay_1ms(){
	  // set gia tri dem cua timer
		Delay_Init();
		TIM_Cmd(TIM2, ENABLE);
		TIM_SetCounter(TIM2, 0);
		while(TIM_GetCounter(TIM2) < 36000);
		TIM_Cmd(TIM2, DISABLE);
}

void Delay_ms(unsigned int _mtime){
	while(_mtime--){
		Delay_1ms();
	}
}