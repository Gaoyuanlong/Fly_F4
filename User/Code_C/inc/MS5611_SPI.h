#ifndef __MS5611_SPI_H__
#define __MS5611_SPI_H__
#include "stm32f4xx.h"
#include "I2C_Soft.h"
#include "Time.h"
#include "Common.h"
#include "arm_math.h"
#include "Filter.h"
struct MS5611_Data_SPI_
{
	int64_t Temperature;
	int64_t Pressure;
	float Altitude;
	float Altitude_Ground;
	float Speed;
};

extern struct MS5611_SPI_
{
	struct MS5611_Data_SPI_* Data; 
	void (*Init)(void);
	void (*Updata)(void);
}MS5611_SPI;

#endif

