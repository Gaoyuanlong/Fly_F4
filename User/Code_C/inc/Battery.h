#ifndef __BATTERY_H__
#define __BATTERY_H__
#include "stm32f4xx.h"
#include "ADS1118.h"
#include "Common.h"
struct Power_VOL_Data_
{
	float BAT_12S;
	float BAT_2S;
	float BAT_1S;
	float Main_5V;
	float Main_3V3_MCU;
	float Main_3V3_SEN;
	float POW_5V;
	float POW_4V5;
	BOOL IsUPS_On; 
};

extern struct  Power_
{
	struct Power_VOL_Data_* Data;
	void (*Init)(void);
	void (*Updata)(void);
}Power;


void ADC_Init();
#endif
