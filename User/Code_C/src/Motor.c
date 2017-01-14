#include "Motor.h"



#define PWM_FREQUENT 500.0f
#define PWM_RANGE   (uint16_t)((1.0f / PWM_FREQUENT) * 1000 * 1000)


void Motor_Init(void);
void Motor_Output(BOOL IsHold);
void Motor_Stop(void);
void Motor_Hold(void);
struct PWM_ PWM_Data;
/*
	Motor÷–PWM£∫1000-2000∑∂Œß
*/
struct Motor_ Motor =
{
	&PWM_Data,
	Motor_Init,
	Motor_Output,
	Motor_Stop,
	Motor_Hold
};

void Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  /* TIM3 clock enable */
	/* GPIOC clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* TIM3 CH1 (PC6), TIM3 CH2 (PC7), TIM3 CH3 (PC8)*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PWM_RANGE;
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock /2) / PWM_FREQUENT / PWM_RANGE) - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM_LIMIT_D;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  /* PWM1 Mode configuration: Channel1 */	
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* TIM3 enable counter */
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
	
	/* TIM4 clock enable */
	/* GPIOC clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  /* TIM4 CH1 (PD12), TIM3 CH2 (PD13), TIM3 CH3 (PD14) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

  /* Connect TIM4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); 

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PWM_RANGE;
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock /2) / PWM_FREQUENT / PWM_RANGE) - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM_LIMIT_D;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  /* PWM1 Mode configuration: Channel1 */	
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  /* TIM4 enable counter */
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
	Motor_Stop();
}

void Motor_Output(BOOL IsHold)
{
	PWM_ PWM_TMP;
	
	if(IsHold == True)
	{
		PWM_TMP.PWM1 = Math.Constrain(PWM_Data.PWM1,PWM_LIMIT_U,PWM_LIMIT_D + THROTTLE_HOLD);
		PWM_TMP.PWM2 = Math.Constrain(PWM_Data.PWM2,PWM_LIMIT_U,PWM_LIMIT_D + THROTTLE_HOLD);
		PWM_TMP.PWM3 = Math.Constrain(PWM_Data.PWM3,PWM_LIMIT_U,PWM_LIMIT_D + THROTTLE_HOLD);
		PWM_TMP.PWM4 = Math.Constrain(PWM_Data.PWM4,PWM_LIMIT_U,PWM_LIMIT_D + THROTTLE_HOLD);
		PWM_TMP.PWM5 = Math.Constrain(PWM_Data.PWM5,PWM_LIMIT_U,PWM_LIMIT_D + THROTTLE_HOLD);
		PWM_TMP.PWM6 = Math.Constrain(PWM_Data.PWM6,PWM_LIMIT_U,PWM_LIMIT_D + THROTTLE_HOLD);
	}
	else
	{
		PWM_TMP.PWM1 = Math.Constrain(PWM_Data.PWM1,PWM_LIMIT_U,PWM_LIMIT_D);
		PWM_TMP.PWM2 = Math.Constrain(PWM_Data.PWM2,PWM_LIMIT_U,PWM_LIMIT_D);
		PWM_TMP.PWM3 = Math.Constrain(PWM_Data.PWM3,PWM_LIMIT_U,PWM_LIMIT_D);
		PWM_TMP.PWM4 = Math.Constrain(PWM_Data.PWM4,PWM_LIMIT_U,PWM_LIMIT_D);
		PWM_TMP.PWM5 = Math.Constrain(PWM_Data.PWM5,PWM_LIMIT_U,PWM_LIMIT_D);
		PWM_TMP.PWM6 = Math.Constrain(PWM_Data.PWM6,PWM_LIMIT_U,PWM_LIMIT_D);

	}
	
	TIM_SetCompare1(TIM3,PWM_TMP.PWM1);
	TIM_SetCompare2(TIM3,PWM_TMP.PWM2);
	TIM_SetCompare3(TIM3,PWM_TMP.PWM3);
	TIM_SetCompare1(TIM4,PWM_TMP.PWM4);
	TIM_SetCompare2(TIM4,PWM_TMP.PWM5);
	TIM_SetCompare3(TIM4,PWM_TMP.PWM6);
}

void Motor_Hold(void)
{
	
	TIM_SetCompare1(TIM3,PWM_LIMIT_D + THROTTLE_HOLD);
	TIM_SetCompare2(TIM3,PWM_LIMIT_D + THROTTLE_HOLD);
	TIM_SetCompare3(TIM3,PWM_LIMIT_D + THROTTLE_HOLD);
	TIM_SetCompare1(TIM4,PWM_LIMIT_D + THROTTLE_HOLD);
	TIM_SetCompare2(TIM4,PWM_LIMIT_D + THROTTLE_HOLD);
	TIM_SetCompare3(TIM4,PWM_LIMIT_D + THROTTLE_HOLD);
}

void Motor_Stop(void)
{
	PWM_Data.PWM1 = PWM_LIMIT_D;
	PWM_Data.PWM2 = PWM_LIMIT_D;
	PWM_Data.PWM3 = PWM_LIMIT_D;
	PWM_Data.PWM4 = PWM_LIMIT_D;
	PWM_Data.PWM5 = PWM_LIMIT_D;
	PWM_Data.PWM6 = PWM_LIMIT_D;
	
	TIM_SetCompare1(TIM3,PWM_LIMIT_D);
	TIM_SetCompare2(TIM3,PWM_LIMIT_D);
	TIM_SetCompare3(TIM3,PWM_LIMIT_D);
	TIM_SetCompare1(TIM4,PWM_LIMIT_D);
	TIM_SetCompare2(TIM4,PWM_LIMIT_D);
	TIM_SetCompare3(TIM4,PWM_LIMIT_D);
}

