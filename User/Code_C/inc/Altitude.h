#ifndef __ALTITUDE_H__
#define __ALTITUDE_H__
#include "MS5611.h"
#include "Math_User.h"
#include "Attitude.h"
#include "Time.h"
#include "Common.h"
#include "Ultra.h"
extern struct Altitude_
{
	float Altitude;
	float Speed;
	void (*Init)(void);
	void (*Updata)(u16 Time);
}Altitude;

#endif
