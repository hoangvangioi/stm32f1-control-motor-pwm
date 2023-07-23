#include "stm32f10x.h"                  // Device header
#include "stm32f10x_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "lcd16x2.h"
#include "delay.h"
#include "stdio.h"

#define PERIOD 1000
int TIM_Pulse = 0;

void GPIO_Configuration();
void EXTI_Configuration();
void Timer_Configuration();
void NVIC_Configuration();


int main()
{
    /* GPIOA, GPIOB, GPIOC, AFIO and TIM1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_Configuration();
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
    EXTI_Configuration();
    Timer_Configuration();
    NVIC_Configuration();

    Delay_Init();
    LCD_Init();

    while (1)
    {
        char buf[16];
        sprintf(buf, "TIM Pulse: %d \r\n", TIM_Pulse);
        if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7) != RESET)
        {
            LCD_Gotoxy(0,0);
            LCD_Puts("IS ACTIVE");
            LCD_Gotoxy(0,1);
            LCD_Puts(buf);
        }
        else
        {
            LCD_Gotoxy(0,0);
            LCD_Puts("SYSTEM");
            LCD_Gotoxy(0,1);
            LCD_Puts("NOT WORKING"); 
        }
    }
}

/* GPIO Configuration */
void GPIO_Configuration()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure PC.13 as Output Push Pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // GPIO A Pin 0, Pin 1, Pin2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Connect EXTI Line0 , Line1, Line2 to PA0, PA1, PA2 pin */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);

    // GPIO A Pin 4 - LED | Pin 6 - IN1 | Pin 7 - EN
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Timer_Configuration()
{
    /* Time base configuration */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Period = PERIOD - 1; // giá trị ARR
    TIM_TimeBaseStructure.TIM_Prescaler = 160 - 1; //giá trị PSC
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;// che do dem xuong
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState =  TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM_Pulse - 1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure); // Chon kenh 1 la chan PA8
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE); // tu dong reload lai gia tri

    /* Cho phep xuat tin hieu o TIM4.*/
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* TIM4 enable counter */
    TIM_Cmd(TIM1, ENABLE);
}

/* EXTI Configuration */
void EXTI_Configuration()
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure EXTI Line0 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

/* NVIC Configuration */
void NVIC_Configuration()
{
    /* Enable and set EXTI Line0 Interrupt to the highest priority */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
}

static void EXTI_Callback(uint8_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_Pin_0)
	{
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0) {}
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, (BitAction)(1 ^ GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4)));
        GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)(1 ^ GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7)));
        if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7) != RESET)
        {
            TIM_Pulse = PERIOD / 2;
        }
        else 
        {
            TIM_Pulse = 0;
        }
	}
	if (GPIO_Pin == GPIO_Pin_1)
	{
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0){}
        if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7) != RESET)
		{
            if (TIM_Pulse < 900) TIM_Pulse = TIM_Pulse + 100;
        }
	}
	if (GPIO_Pin == GPIO_Pin_2)
	{
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 0){}
        if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7) != RESET)
        {
            if (TIM_Pulse > 100) TIM_Pulse = TIM_Pulse - 100;
        }
	}
    TIM_SetCompare1(TIM1, TIM_Pulse);
}

void EXTI0_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_Callback(0x0001);
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

void EXTI1_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        EXTI_Callback(0x0002);
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void EXTI2_IRQHandler()
{    
    if (EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        EXTI_Callback(0x0004);
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}