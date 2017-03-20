#include "Communication.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
#define DEBUG

BOOL Communicate(void);

struct User_Data_ User_Data;

struct Communication_ Communication = 
{
	Communicate
};

void Send_Eular_MCU2(void)
{
	if(USART.Request_TXBUF() == True)
	{
		u8 Cnt = 0;
		u16 Temp1 = 0;
		
		
		USART.Data->TX_BUF[Cnt++] = 0XAA;
		USART.Data->TX_BUF[Cnt++] = 0X55;
		USART.Data->TX_BUF[Cnt++] = 0X02;
		Cnt++;	
		
		Temp1 = (int)(Attitude.Angle->x * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);	
		
		Temp1 = (int)(Attitude.Angle->y * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);	

		Temp1 = (int)(Attitude.Angle->z * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1); 
		
		Temp1 = (int)MPU6050.Data->ACC_ADC.x;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		Temp1 = (int)MPU6050.Data->ACC_ADC.y;	
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		Temp1 = (int)MPU6050.Data->ACC_ADC.z;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)MPU6050.Data->GYR_ADC.x;	
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)MPU6050.Data->GYR_ADC.y;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)MPU6050.Data->GYR_ADC.z;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)HMC5883.Data->MAG_ADC.x;	
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)HMC5883.Data->MAG_ADC.y;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)HMC5883.Data->MAG_ADC.z;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		USART.Data->TX_BUF[3] = Cnt;
		
		u8 Sum = 0;
		for(u8 i = 0;i < Cnt; i++)
			Sum += USART.Data->TX_BUF[i];
		USART.Data->TX_BUF[Cnt] = Sum;
			
		USART.Send();	
	}
}

void Send_Battery_MS5611_MCU2(void)
{
	if(USART.Request_TXBUF() == True)
	{	
		u8 Cnt = 0;
		u32 Temp1 = 0;
				
		USART.Data->TX_BUF[Cnt++] = 0XAA;
		USART.Data->TX_BUF[Cnt++] = 0X55;
		USART.Data->TX_BUF[Cnt++] = 0X03;
		Cnt++;	
		
		Temp1 = (int)(Power.Data->Main_5V * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);	
		
		Temp1 = (int)(Power.Data->Main_3V3_MCU * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);	

		Temp1 = (int)(Power.Data->Main_3V3_SEN * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1); 
		
		Temp1 = (int)(Power.Data->BAT_2S * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		Temp1 = (int)(Power.Data->BAT_1S * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		Temp1 = (int)(Power.Data->POW_5V * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)(Power.Data->POW_4V5 * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)(Power.Data->BAT_12S * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)(Position.Data->POS_Z * 100.0f);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)(MS5611_SPI.Data->Temperature);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = (int)(MS5611_SPI.Data->Pressure);
		USART.Data->TX_BUF[Cnt++] = BYTE3(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE2(Temp1);
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		USART.Data->TX_BUF[3] = Cnt;
		
		u8 Sum = 0;
		for(u8 i = 0;i < Cnt; i++)
			Sum += USART.Data->TX_BUF[i];
		USART.Data->TX_BUF[Cnt] = Sum;
		
		USART.Send();	
	}
}

void Send_Motor_State_MCU2(void)
{
	if(USART.Request_TXBUF() == True)
	{
		u8 Cnt = 0;
		u32 Temp1 = 0;
		
		
		USART.Data->TX_BUF[Cnt++] = 0XAA;
		USART.Data->TX_BUF[Cnt++] = 0X55;
		USART.Data->TX_BUF[Cnt++] = 0X04;
		Cnt++;	
		
		Temp1 = Motor.PWM->PWM1;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);	
		
		Temp1 = Motor.PWM->PWM2;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);	

		Temp1 = Motor.PWM->PWM3;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1); 
		
		Temp1 = Motor.PWM->PWM4;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		Temp1 = Motor.PWM->PWM5;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		Temp1 = Motor.PWM->PWM6;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		USART.Data->TX_BUF[3] = Cnt;
		
		u8 Sum = 0;
		for(u8 i = 0;i < Cnt; i++)
			Sum += USART.Data->TX_BUF[i];
		USART.Data->TX_BUF[Cnt] = Sum;
			
		USART.Send();	
		
	}
}

void Send_RC_MCU2(void)
{
	if(USART.Request_TXBUF() == True)
	{	
		u8 Cnt = 0;
		u16 Temp1 = 0;
		
		
		USART.Data->TX_BUF[Cnt++] = 0XAA;
		USART.Data->TX_BUF[Cnt++] = 0X55;
		USART.Data->TX_BUF[Cnt++] = 0X01;
		Cnt++;	
		
		Temp1 = RC_THROTTLE;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);	
		
		Temp1 = RC_YAW;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);	

		Temp1 = RC_PITCH;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		Temp1 = RC_ROLL;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		Temp1 = RC_AUX1;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		Temp1 = RC_AUX2;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = RC_AUX3;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = RC_AUX4;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = RC_AUX5;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		Temp1 = RC_AUX6;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);	

		Temp1 = RC_AUX7;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);

		Temp1 = RC_AUX8;
		USART.Data->TX_BUF[Cnt++] = BYTE1(Temp1);	
		USART.Data->TX_BUF[Cnt++] = BYTE0(Temp1);
		
		USART.Data->TX_BUF[3] = Cnt;
		
		u8 Sum = 0;
		for(u8 i = 0;i < Cnt; i++)
			Sum += USART.Data->TX_BUF[i];
		USART.Data->TX_BUF[Cnt] = Sum;
			
		USART.Send();
	}
}

void Send_Senser_PC(void)
{
	vs16 Temp1 = 0;
	vs32 Temp2 = 0;
	u8 Cnt = 1;
	if(NRF24L01.RequsetAckBuf(False) == False) return;	
	
	NRF24L01.Data->ACK_BUF[Cnt++] = 0XAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0XAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0X02;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0;
	
	Temp1 = MPU6050.Data->ACC_ADC.x;	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = MPU6050.Data->ACC_ADC.y;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = MPU6050.Data->ACC_ADC.z;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = MPU6050.Data->GYR_ADC.x;	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = MPU6050.Data->GYR_ADC.y;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = MPU6050.Data->GYR_ADC.z;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);
	
	Temp1 = HMC5883.Data->MAG_Original.x;//地磁X
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);

	Temp1 = HMC5883.Data->MAG_Original.y;//地磁Y
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);
	
	Temp1 = HMC5883.Data->MAG_Original.z;//地磁Z
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);

	NRF24L01.Data->ACK_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i = 1;i < Cnt; i++)
		Sum += NRF24L01.Data->ACK_BUF[i];
	
	NRF24L01.Data->ACK_BUF[Cnt++] = Sum;
	NRF24L01.Data->ACK_BUF[0] = Cnt - 1;   

	NRF24L01.SendData();

	
}

void Send_Eular_PC(void)
{
	u16 Temp1 = 0;
	u32 Temp2 = 0;
	u8 Cnt = 1;

	if(NRF24L01.RequsetAckBuf(False) == False) return;	
	
	NRF24L01.Data->ACK_BUF[Cnt++] = 0XAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0XAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0X01;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0;
	
	Temp1 = (int)(Attitude.Angle->y * 100.0f);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = (int)(Attitude.Angle->x * 100.0f);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = (int)(Attitude.Angle->z * 100.0f);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp2 = Position.Data->POS_Z * 100.0f;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE3(Temp2);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE2(Temp2);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp2);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp2);
	
	NRF24L01.Data->ACK_BUF[Cnt++] = MPU6050.IsCalibrating;
	NRF24L01.Data->ACK_BUF[Cnt++] = True;	

	NRF24L01.Data->ACK_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i = 1;i < Cnt; i++)
		Sum += NRF24L01.Data->ACK_BUF[i];
	
	NRF24L01.Data->ACK_BUF[Cnt++] = Sum;
	NRF24L01.Data->ACK_BUF[0] = Cnt - 1; 
	NRF24L01.SendData();
	
}

void Send_RC_PC(void)
{
	u16 Temp1 = 0;
	u8 Cnt = 1;

	if(NRF24L01.RequsetAckBuf(False) == False) return;	
	
	NRF24L01.Data->ACK_BUF[Cnt++] = 0XAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0XAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0X03;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0;
	
	Temp1 = RC_THROTTLE ;	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = RC_YAW;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = RC_ROLL;	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = RC_PITCH;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);

	Temp1 = RC_AUX1;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);

	Temp1 = RC_AUX2;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);
	
	Temp1 = RC_AUX3;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);	

	Temp1 = RC_AUX4;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);
	
	Temp1 = RC_AUX5;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);

	Temp1 = RC_AUX6;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);

	NRF24L01.Data->ACK_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i = 1;i < Cnt; i++)
		Sum += NRF24L01.Data->ACK_BUF[i];
	
	NRF24L01.Data->ACK_BUF[Cnt++] = Sum;
	NRF24L01.Data->ACK_BUF[0] = Cnt - 1; 
	NRF24L01.SendData();
}

void Send_Motor_PC(void)
{
	u16 Temp1 = 0;
	u8 Cnt = 1;

	if(NRF24L01.RequsetAckBuf(False) == False) return;	
	
	NRF24L01.Data->ACK_BUF[Cnt++] = 0XAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0XAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0X06;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0;
	
	Temp1 = Motor.PWM->PWM1;	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = Motor.PWM->PWM2;	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = Motor.PWM->PWM3;	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = Motor.PWM->PWM4;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);

	Temp1 = Motor.PWM->PWM5;;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);

	Temp1 = Motor.PWM->PWM6;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);

	Temp1 = 0;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);
	
	Temp1 = 0;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);	

	NRF24L01.Data->ACK_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i = 1;i < Cnt; i++)
		Sum += NRF24L01.Data->ACK_BUF[i];
	
	NRF24L01.Data->ACK_BUF[Cnt++] = Sum;
	NRF24L01.Data->ACK_BUF[0] = Cnt - 1; 
	NRF24L01.SendData();
}
BOOL Send_PID_Para_PC(u8 Group)
{
	u8 Cnt = 1;
	vs16 Temp = 0;
	PID Temp_PID[3];
	
	if(NRF24L01.RequsetAckBuf(True) == False) return False;	
	
	switch(Group)
	{
		case 1:
			Temp_PID[0].Kp = FlyControl.Para->ATT_Outer_PID_x.Kp;
			Temp_PID[0].Ki = FlyControl.Para->ATT_Outer_PID_x.Ki;
			Temp_PID[0].Kd = FlyControl.Para->ATT_Outer_PID_x.Kd;
		
			Temp_PID[1].Kp = FlyControl.Para->ATT_Outer_PID_y.Kp;
			Temp_PID[1].Ki = FlyControl.Para->ATT_Outer_PID_y.Ki;
			Temp_PID[1].Kd = FlyControl.Para->ATT_Outer_PID_y.Kd;
		
			Temp_PID[2].Kp = FlyControl.Para->ATT_Outer_PID_z.Kp;
			Temp_PID[2].Ki = FlyControl.Para->ATT_Outer_PID_z.Ki;
			Temp_PID[2].Kd = FlyControl.Para->ATT_Outer_PID_z.Kd;
		break;	
		case 2:
			Temp_PID[0].Kp = FlyControl.Para->POS_Outer_PID_x.Kp;
			Temp_PID[0].Ki = FlyControl.Para->POS_Outer_PID_x.Ki;
			Temp_PID[0].Kd = FlyControl.Para->POS_Outer_PID_x.Kd;
		
			Temp_PID[1].Kp = FlyControl.Para->POS_Outer_PID_y.Kp;
			Temp_PID[1].Ki = FlyControl.Para->POS_Outer_PID_y.Ki;
			Temp_PID[1].Kd = FlyControl.Para->POS_Outer_PID_y.Kd;
		
			Temp_PID[2].Kp = FlyControl.Para->POS_Outer_PID_z.Kp;
			Temp_PID[2].Ki = FlyControl.Para->POS_Outer_PID_z.Ki;
			Temp_PID[2].Kd = FlyControl.Para->POS_Outer_PID_z.Kd;			
			break;
		case 3:
			Temp_PID[0].Kp = FlyControl.Para->ATT_Inner_PID_x.Kp;
			Temp_PID[0].Ki = FlyControl.Para->ATT_Inner_PID_x.Ki;
			Temp_PID[0].Kd = FlyControl.Para->ATT_Inner_PID_x.Kd;
		
			Temp_PID[1].Kp = FlyControl.Para->ATT_Inner_PID_y.Kp;
			Temp_PID[1].Ki = FlyControl.Para->ATT_Inner_PID_y.Ki;
			Temp_PID[1].Kd = FlyControl.Para->ATT_Inner_PID_y.Kd;
		
			Temp_PID[2].Kp = FlyControl.Para->ATT_Inner_PID_z.Kp;
			Temp_PID[2].Ki = FlyControl.Para->ATT_Inner_PID_z.Ki;
			Temp_PID[2].Kd = FlyControl.Para->ATT_Inner_PID_z.Kd;
			break;
		case 4:
			Temp_PID[0].Kp = FlyControl.Para->POS_Inner_PID_x.Kp;
			Temp_PID[0].Ki = FlyControl.Para->POS_Inner_PID_x.Ki;
			Temp_PID[0].Kd = FlyControl.Para->POS_Inner_PID_x.Kd;
		
			Temp_PID[1].Kp = FlyControl.Para->POS_Inner_PID_y.Kp;
			Temp_PID[1].Ki = FlyControl.Para->POS_Inner_PID_y.Ki;
			Temp_PID[1].Kd = FlyControl.Para->POS_Inner_PID_y.Kd;
		
			Temp_PID[2].Kp = FlyControl.Para->POS_Inner_PID_z.Kp;
			Temp_PID[2].Ki = FlyControl.Para->POS_Inner_PID_z.Ki;
			Temp_PID[2].Kd = FlyControl.Para->POS_Inner_PID_z.Kd;
			break;		
		default:
			break;
	}
	
	NRF24L01.Data->ACK_BUF[Cnt++] = 0xAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0xAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0x10 + Group - 1;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0;
	
	Temp = Temp_PID[0].Kp;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[0].Ki;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[0].Kd;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = Temp_PID[1].Kp;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[1].Ki;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[1].Kd;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = Temp_PID[2].Kp;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[2].Ki;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[2].Kd;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	
	NRF24L01.Data->ACK_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i=1;i<Cnt;i++)
		Sum += NRF24L01.Data->ACK_BUF[i];
	
	NRF24L01.Data->ACK_BUF[Cnt++]=Sum;

	NRF24L01.Data->ACK_BUF[0] = Cnt - 1;    //HID第一个字节 表示数据长度
	
	NRF24L01.SendData();
	return True;
}

void Send_Location_PC(void)
{
	u16 Temp1 = 0;
	u8 Cnt = 1;

	if(NRF24L01.RequsetAckBuf(False) == False) return;	
	
	NRF24L01.Data->ACK_BUF[Cnt++] = 0XAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0XAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0X32;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0;
	
	Temp1 = (int)(Position.Data->POS_X*100)%10000;	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = (int)(Position.Data->POS_Y*100)%10000;	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = (int)(Position.Data->POS_Z*100)%10000;	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp1);	
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp1);		
	
	NRF24L01.Data->ACK_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i = 1;i < Cnt; i++)
		Sum += NRF24L01.Data->ACK_BUF[i];
	
	NRF24L01.Data->ACK_BUF[Cnt++] = Sum;
	NRF24L01.Data->ACK_BUF[0] = Cnt - 1; 
	NRF24L01.SendData();
}

//上传数据卡顿或出错
// EEROR_LQ
BOOL Send_UserData_PC(void)
{
	u8 Cnt = 1;
	vs16 Temp = 0;
	
	if(NRF24L01.RequsetAckBuf(True) == False) return False;	
	
	NRF24L01.Data->ACK_BUF[Cnt++] = 0xAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0xAA;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0xF1;
	NRF24L01.Data->ACK_BUF[Cnt++] = 0;
	
	Temp = User_Data.Data1;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = (int)(User_Data.Data2*100)%10000;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = (int)(User_Data.Data3*100)%10000;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);

	Temp = (int)(User_Data.Data4*100)%10000;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);

	Temp = User_Data.Data5;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = (int)(User_Data.Data6*100)%10000;
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE1(Temp);
	NRF24L01.Data->ACK_BUF[Cnt++] = BYTE0(Temp);	

	NRF24L01.Data->ACK_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i=1;i<Cnt;i++)
		Sum += NRF24L01.Data->ACK_BUF[i];
	
	NRF24L01.Data->ACK_BUF[Cnt++]=Sum;

	NRF24L01.Data->ACK_BUF[0] = Cnt - 1;    //HID第一个字节 表示数据长度
	
	NRF24L01.SendData();
	return True;
}

void Send_Reply_PC(u16 Sum,u8 Head)
{
	
	if(NRF24L01.RequsetAckBuf(True) == False) return;	      //数据帧被占用，停止向数据帧写入数据
	
	NRF24L01.Data->ACK_BUF[0] = 0X07;                      //帧长
	NRF24L01.Data->ACK_BUF[1] = 0XAA;
	NRF24L01.Data->ACK_BUF[2] = 0XAA;
	NRF24L01.Data->ACK_BUF[3] = 0XEF;
	NRF24L01.Data->ACK_BUF[4] = 2;
	NRF24L01.Data->ACK_BUF[5] = Head;
	
	NRF24L01.Data->ACK_BUF[6] = Sum;	
	
	u8 sum = 0;
	for(u8 i=1;i<7;i++)
		sum += NRF24L01.Data->ACK_BUF[i];
	
	NRF24L01.Data->ACK_BUF[7]=sum;	
	
	NRF24L01.SendData();
}

void Send_Data_PC(void)
{
	static u8 Cnt = 0;

	switch (Cnt++ % 5) 
	{
		case 0:
			Send_Eular_PC();
			break;
		case 1:
			Send_Senser_PC();
			break;
		case 2:
			Send_RC_PC();
			break;
		case 3:
			Send_Location_PC();
			break;
		case 4:
			Send_UserData_PC();
			break;
		default:
			break;
	}
}

void Data_Analysis_PC(void)
{
	u8 i = 0;
	u8 Start = 0;
	u8 Length = 0;
	u8 Sum = 0;
	u8 Funtion = 0;
	if(NRF24L01.GetData() == True) 
	{
		for(i = 0;i < 32; i++)
		{
			if(i == 31) return;
			if(NRF24L01.Data->RX_BUF[i] == 0XAA & NRF24L01.Data->RX_BUF[i+1] == 0XAF)
				break;
		}
		
		Length = NRF24L01.Data->RX_BUF[i + 3] + 4;
		Start = i;
		Funtion = NRF24L01.Data->RX_BUF[i + 2];
		
		for(i = 0;i < Length;i++)
			Sum += NRF24L01.Data->RX_BUF[Start + i];
		
		if(Sum == NRF24L01.Data->RX_BUF[Start + Length])
		{ 
		 switch (Funtion) 																      //帧功能分析
			{

				case 0X02:
					static u8 PID_Group = 1;
					
					switch (PID_Group)
					{
						case 1:
							Send_PID_Para_PC(1);
							break;
						case 2:
							Send_PID_Para_PC(2);
							break;
						case 3:
							Send_PID_Para_PC(3);
							break;
						case 4:
							Send_PID_Para_PC(4);
							break;
						default:
							break;
					}
			
					if(++PID_Group == 5) PID_Group = 1;
					
					break;
				case 0X10:                                           //PID1 2用于描述外环参数
					FlyControl.Para->ATT_Outer_PID_x.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 4] << 8 | NRF24L01.Data->RX_BUF[Start + 5]);
					FlyControl.Para->ATT_Outer_PID_x.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 6] << 8 | NRF24L01.Data->RX_BUF[Start + 7]);
					FlyControl.Para->ATT_Outer_PID_x.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 8] << 8 | NRF24L01.Data->RX_BUF[Start + 9]);

					FlyControl.Para->ATT_Outer_PID_y.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 10] << 8 | NRF24L01.Data->RX_BUF[Start + 11]);
					FlyControl.Para->ATT_Outer_PID_y.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 12] << 8 | NRF24L01.Data->RX_BUF[Start + 13]);
					FlyControl.Para->ATT_Outer_PID_y.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 14] << 8 | NRF24L01.Data->RX_BUF[Start + 15]);
				
					FlyControl.Para->ATT_Outer_PID_z.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 16] << 8 | NRF24L01.Data->RX_BUF[Start + 17]);
					FlyControl.Para->ATT_Outer_PID_z.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 18] << 8 | NRF24L01.Data->RX_BUF[Start + 19]);
					FlyControl.Para->ATT_Outer_PID_z.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 20] << 8 | NRF24L01.Data->RX_BUF[Start + 21]);		
		
					Send_Reply_PC(Sum,0X10);
				
					break;
				case 0X11:
					FlyControl.Para->POS_Outer_PID_x.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 4] << 8 | NRF24L01.Data->RX_BUF[Start + 5]);
					FlyControl.Para->POS_Outer_PID_x.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 6] << 8 | NRF24L01.Data->RX_BUF[Start + 7]);
					FlyControl.Para->POS_Outer_PID_x.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 8] << 8 | NRF24L01.Data->RX_BUF[Start + 9]);

					FlyControl.Para->POS_Outer_PID_y.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 10] << 8 | NRF24L01.Data->RX_BUF[Start + 11]);
					FlyControl.Para->POS_Outer_PID_y.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 12] << 8 | NRF24L01.Data->RX_BUF[Start + 13]);
					FlyControl.Para->POS_Outer_PID_y.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 14] << 8 | NRF24L01.Data->RX_BUF[Start + 15]);
				
					FlyControl.Para->POS_Outer_PID_z.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 16] << 8 | NRF24L01.Data->RX_BUF[Start + 17]);
					FlyControl.Para->POS_Outer_PID_z.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 18] << 8 | NRF24L01.Data->RX_BUF[Start + 19]);
					FlyControl.Para->POS_Outer_PID_z.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 20] << 8 | NRF24L01.Data->RX_BUF[Start + 21]);	
				
					Send_Reply_PC(Sum,0X11);
				
					break;
				case 0X12:                                         //PID7 8 9用于描述内环参数
					FlyControl.Para->ATT_Inner_PID_x.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 4] << 8 | NRF24L01.Data->RX_BUF[Start + 5]);
					FlyControl.Para->ATT_Inner_PID_x.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 6] << 8 | NRF24L01.Data->RX_BUF[Start + 7]);
					FlyControl.Para->ATT_Inner_PID_x.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 8] << 8 | NRF24L01.Data->RX_BUF[Start + 9]);

					FlyControl.Para->ATT_Inner_PID_y.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 10] << 8 | NRF24L01.Data->RX_BUF[Start + 11]);
					FlyControl.Para->ATT_Inner_PID_y.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 12] << 8 | NRF24L01.Data->RX_BUF[Start + 13]);
					FlyControl.Para->ATT_Inner_PID_y.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 14] << 8 | NRF24L01.Data->RX_BUF[Start + 15]);
				
					FlyControl.Para->ATT_Inner_PID_z.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 16] << 8 | NRF24L01.Data->RX_BUF[Start + 17]);
					FlyControl.Para->ATT_Inner_PID_z.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 18] << 8 | NRF24L01.Data->RX_BUF[Start + 19]);
					FlyControl.Para->ATT_Inner_PID_z.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 20] << 8 | NRF24L01.Data->RX_BUF[Start + 21]);
				
					Send_Reply_PC(Sum,0X12);
				
					break;
				case 0X13:                                         //PID7 8 9用于描述内环参数
					FlyControl.Para->POS_Inner_PID_x.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 4] << 8 | NRF24L01.Data->RX_BUF[Start + 5]);
					FlyControl.Para->POS_Inner_PID_x.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 6] << 8 | NRF24L01.Data->RX_BUF[Start + 7]);
					FlyControl.Para->POS_Inner_PID_x.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 8] << 8 | NRF24L01.Data->RX_BUF[Start + 9]);

					FlyControl.Para->POS_Inner_PID_y.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 10] << 8 | NRF24L01.Data->RX_BUF[Start + 11]);
					FlyControl.Para->POS_Inner_PID_y.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 12] << 8 | NRF24L01.Data->RX_BUF[Start + 13]);
					FlyControl.Para->POS_Inner_PID_y.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 14] << 8 | NRF24L01.Data->RX_BUF[Start + 15]);
				
					FlyControl.Para->POS_Inner_PID_z.Kp = (float)(NRF24L01.Data->RX_BUF[Start + 16] << 8 | NRF24L01.Data->RX_BUF[Start + 17]);
					FlyControl.Para->POS_Inner_PID_z.Ki = (float)(NRF24L01.Data->RX_BUF[Start + 18] << 8 | NRF24L01.Data->RX_BUF[Start + 19]);
					FlyControl.Para->POS_Inner_PID_z.Kd = (float)(NRF24L01.Data->RX_BUF[Start + 20] << 8 | NRF24L01.Data->RX_BUF[Start + 21]);
				
					Send_Reply_PC(Sum,0X13);
				
					break;				
				default:
					break;
			}
		}
	
		NRF24L01.Free_RXBUF();
	}
}

void Send_Data_MCU2(void)
{
	static u8 Mark = 0;
	switch (Mark++ % 4)
	{
		case 0:
			Send_Eular_MCU2();	
			break;
		case 1:
			Send_Battery_MS5611_MCU2();	
			break;
		case 2:
			Send_Motor_State_MCU2();
			break;
		case 3:
			Send_RC_MCU2();
			break;
		default:
			break;
	}
	
}

void Receive_Data_MCU2( const u8 *StrHeadAdd)
{
	 const u8 *P = StrHeadAdd;
	
	if(USART.Receive()== True)
	{
		if(P[0] == 'G' && P[1] == 'P' && P[2] == 'S' && P[3] == 'A')
		{
			RTK_OPS.Read((u8*)&P[4]);
		}
		USART.Free_RXBUF();
	}
}

void Data_Analysis_MCU2(void)
{
	u8 i = 0;
	u8 Start = 0;
	u8 Length = 0;
	u8 Sum = 0;
	u8 Funtion = 0;
	u8 Data[32];
	if(USART.Receive() == True)
	{
		for(i = 0;i < 32; i++)
		{
			Data[i] = USART.Data->RX_BUF[i];
		}
		USART.Free_RXBUF();
		for(i = 0;i < 32; i++)
		{
			if(i == 31) return;
			if(Data[i] == 0XAA & Data[i+1] == 0X55)
				break;
		}
		Start = i;
		Funtion = Data[i + 2];
		Length = Data[i + 3];
					
		for(i = 0;i < Length;i++)
			Sum += Data[Start + i];
		
		if(Sum == Data[Start + Length])
		{ 
		 switch (Funtion) 																      
			{
				case 0X01:   

					break;
				case 0X02:
					Ultra.Altitude = (int16_t)(Data[Start + 4] << 8 | Data[Start + 5]) / 100.0f;
					Ultra.Speed    = (int16_t)(Data[Start + 6] << 8 | Data[Start + 7]) / 100.0f;
					break;
				case 0X03:

					break;
				case 0X04:  

					break;
				case 0X05:
					
					break;
				case 0X06:  
					
					break;
				default:
					break;
			}
		}	
	}
}

BOOL Communicate(void)
{
//	Send_Data_MCU2();
#ifdef DEBUG
	//Send_Data_PC();
	//ANO_DT_Data_Exchange();
	USART1_Cf.USART1_Send_Datas();
	//Data_Analysis_PC();
#endif	
//	Data_Analysis_MCU2();
	Receive_Data_MCU2(USART.Data->RX_BUF);
	return True;
}

