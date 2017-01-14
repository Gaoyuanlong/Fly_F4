#ifndef __POSITION_H__
#define __POSITION_H__

#include "MS5611_SPI.h"
#include "Math_User.h"
#include "Attitude.h"
#include "Time.h"
#include "Common.h"
#include "Ultra.h"
#include "Buzzer.h"
struct Pos_Data_
{
	float POS_X;
	float POS_Y;
	float POS_Z;
	float SPE_X;
	float SPE_Y;
	float SPE_Z;
};

extern struct Position_
{
	Pos_Data_* Data;
	BOOL IsReady;
	void (*Init)(void);
	void (*Updata)(u16 Time);
}Position;


#endif
