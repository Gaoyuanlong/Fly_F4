#ifndef __FLYCONTROL__H
#define __FLYCONTROL__H
#include "PID.h"
#include "Attitude.h"
#include "Common.h"
#include "SBUS.h"
#include "Motor.h"
#include "HMC5883.h"
#include "Position.h"
#define THROTTLE_MIN SBUS_MIN
#define THROTTLE_MAX SBUS_MAX
#define THROTTLE_MID ((SBUS_MAX + SBUS_MIN) / 2)
#define THROTTLE_80_PERCENT (((THROTTLE_MAX - THROTTLE_MIN) * 0.8f) + THROTTLE_MIN)

enum Fly_Mode
{
	ATT,
	ALT,
	POS,
};

struct Control_Para_
{
	PID ATT_Inner_PID_x;
	PID ATT_Inner_PID_y;
	PID ATT_Inner_PID_z;
	
	PID ATT_Outer_PID_x;
	PID ATT_Outer_PID_y;	
	PID ATT_Outer_PID_z;	
	
	PID POS_Inner_PID_x;
	PID POS_Inner_PID_y;
	PID POS_Inner_PID_z;
	
	PID POS_Outer_PID_x;
	PID POS_Outer_PID_y;
	PID POS_Outer_PID_z;

	float POS_ORI_X;
	float POS_ORI_Y;
	float POS_ORI_Z;

	float SPE_ORI_X;
	float SPE_ORI_Y;
	float SPE_ORI_Z;

	BOOL IsLock;
	BOOL IsLost;
	Fly_Mode Mode;
};

extern struct FlyControl_
{
	struct Control_Para_* Para;

	void (*ATT_InnerLoop)(u32 Time);
	void (*ATT_OuterLoop)(u32 Time);
	
	void (*POS_InnerLoop)(u32 Time);
	void (*POS_OuterLoop)(u32 Time);
	
}FlyControl;

#endif
