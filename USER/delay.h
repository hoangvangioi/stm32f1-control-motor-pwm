#ifndef __DELAY_H
#define __DELAY_H


#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM

void Delay_Init();
void Delay_1us();
void Delay_us(unsigned int _utime);
void Delay_1ms();
void Delay_ms(unsigned int _mtime);

#endif
