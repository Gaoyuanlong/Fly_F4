#ifndef __SYS_STATE_H__
#define __SYS_STATE_H__
#include "Board.h"

extern struct SystemState_
{
	BOOL IsSensorReady;
	BOOL IsPowerALow;
	BOOL IsPowerBLow;
	BOOL IsPowerError;
	void (*Updata)(u16 Time);
}SystemState;


#endif
