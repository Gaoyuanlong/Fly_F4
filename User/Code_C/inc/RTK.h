#ifndef __RTK_H__
#define __RTK_H__
#include "USART.h"
#include "Filter.h"

#define RTK_GPS_SIZE 48
#define GPS_HEAD_SIZE 3

typedef struct RTK_GPS_
{
	u8 Quality;
	double Lat_M;
	double Lon_M;
	double Alt_M;
	double TrackAngle;
	double Speed_M;
}RTK_GPS_;

typedef struct RTK_OPS_
{
	BOOL (*Read)( u8* StrAdd);
}RTK_OPS_;

extern RTK_GPS_ RTK_GPS;
extern RTK_OPS_ RTK_OPS;
extern struct Vector RTK_GPS_Speed; 
#endif

