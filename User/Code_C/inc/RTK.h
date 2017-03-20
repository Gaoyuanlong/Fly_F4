#ifndef __RTK_H__
#define __RTK_H__
#include "USART.h"
#include "Filter.h"

#define RTK_GPS_SIZE 96
#define GPS_HEAD_SIZE 4


typedef struct RTK_XYZ_HP_
{
	double PX;
	double PY;
	double PZ;
	
	double VX;
	double VY;
	double VZ;
	
	double Heading;
	double Pitch;
	
	double Lon_M;
	double Lat_M;
	double Alt_M;
	u8 Quality;
}RTK_XYZ_HP_;

typedef struct RTK_OPS_
{
	BOOL (*Read)( u8* StrAdd);
}RTK_OPS_;

extern RTK_XYZ_HP_ RTK_XYZ_HP;
extern RTK_XYZ_HP_ RTK_XYZ_HP_Offset;
extern RTK_OPS_ RTK_OPS;
extern struct Vector RTK_GPS_Speed; 
#endif

