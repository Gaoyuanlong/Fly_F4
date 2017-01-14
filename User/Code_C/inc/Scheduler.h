#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__
#include "Board.h"

extern struct Scheduler_ 
{
	void (*Loop_1000Hz)(u16 Time);
	void (*Loop_500Hz)(u16 Time);
	void (*Loop_200Hz)(u16 Time);
	void (*Loop_50Hz)(u16 Time);
}Scheduler;

#endif
