#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__
#include "stm32f4xx.h"
#include "Common.h"
#include "USART.h"
#include "Attitude.h"
#include "MPU6050.h"
#include "MS5611.h"
#include "Battery.h"
#include "Motor.h"
#include "GPS.h"
#include "NRF24L01.h"
#include "HMC5883.h"
#include "FlyControl.h"
#include "SBUS.h"
#include "Altitude.h"
#include "Ultra.h"

#define UNLOCK  	0X0001
#define LOST    	0X0002
#define SYS_ERROR 0X0004
#define POW_LOW 	0X0008
#define POW_LOS 	0X0010
#define MAG_ERROR 0X0020
#define UPS_ON    0X0040
struct SYS_State_
{
	u16 State1;
	u16 State2;
	u16 State3;
};

struct User_Data_
{
	float Data1;
	float Data2;
	float Data3;
	float Data4;
	float Data5;
	float Data6;
};
extern struct User_Data_ User_Data;
extern struct SYS_State_ SYS_State;
extern struct Communication_
{
	BOOL (*UpData)(void);

}Communication;

#endif
