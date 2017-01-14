#include "Board.h"

Target_ TG_50HZ(Scheduler.Loop_50Hz,20000);
Target_ TG_200HZ(Scheduler.Loop_200Hz,5000);
Target_ TG_500HZ(Scheduler.Loop_500Hz,2000);
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  
	SystemTime.Init();
	Led.Init();
	USART.Init(256000);
	Power.Init();
	SBUS.Init();
	MPU6050.Init();
	NRF24L01.Init(RXMode);
	Motor.Init();
	HMC5883.Init();
	Position.Init();
	Buzzer.Init();
	while(1)
	{		
		TG_50HZ.Run();
		TG_200HZ.Run();
		TG_500HZ.Run();
	}
}

