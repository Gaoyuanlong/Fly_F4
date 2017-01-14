#include "Altitude.h"

void Altitude_Updata(u16 Time);
void Altitude_Init(void);
Filter_Balance FIL_Speed(0.1,0);
Filter_Balance FIL_Altitude(1,0);
struct Altitude_ Altitude = 
{
	9,
	0,
	Altitude_Init,
	Altitude_Updata
};

//Time :MS

void Altitude_Init(void)
{
	MS5611.Init();
}

Vector ACC_Earth;
void Altitude_Updata(u16 Time)
{
	
	float Speed;	
	float Alt = 0;

	Alt = Ultra.Altitude;
	Speed = Ultra.Speed;

//	Alt = MS5611.Data->Altitude;
//	Speed = MS5611.Data->Speed;
	
	ACC_Earth = Math.Body_To_Earth(MPU6050.Data->ACC_ADC,Attitude.Angle->y,Attitude.Angle->x);
	
	ACC_Earth.z -= 4095;
	ACC_Earth.z /= 4095;
	ACC_Earth.z *= 9.8f;
	
	Speed = FIL_Speed.BalanceFilter(ACC_Earth.z,Speed,(float)Time / 1000);
	Alt = FIL_Altitude.BalanceFilter(Speed,Alt,(float)Time / 1000);
	
	Altitude.Speed = (Alt - Altitude.Altitude) / Time * (float)1e3;
	Altitude.Altitude = Alt;

	User_Data.Data1 = Altitude.Altitude * 100;
}

