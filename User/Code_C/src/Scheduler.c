#include "Scheduler.h"
void Loop_1000Hz(u16 Time);
void Loop_500Hz(u16 Time);
void Loop_200Hz(u16 Time);
void Loop_50Hz(u16 Time);

struct Scheduler_ Scheduler = 
{
	Loop_1000Hz,
	Loop_500Hz,
	Loop_200Hz,
	Loop_50Hz
};

void Loop_1000Hz(u16 Time)
{
	
}

void Loop_500Hz(u16 Time)
{
	MPU6050.Updata();
	Attitude.Updata();
	Position.Updata(Time);
	FlyControl.ATT_InnerLoop(Time);
	
}

void Loop_200Hz(u16 Time)
{
	SBUS.Updata();
	HMC5883.Updata();
	FlyControl.ATT_OuterLoop(Time);	
	FlyControl.POS_InnerLoop(Time);
}

void Loop_50Hz(u16 Time)
{
	SystemState.Updata(Time);
	Communication.UpData();
	
	FlyControl.POS_OuterLoop(Time);
	
	Power.Updata();	
	MS5611_SPI.Updata();
	GPS.Updata();

	
}
