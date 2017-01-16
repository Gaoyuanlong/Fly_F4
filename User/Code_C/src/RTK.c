#include "RTK.h"


BOOL RTK_Read( u8 *StrAdd);

RTK_GPS_ RTK_GPS;

RTK_OPS_ RTK_OPS=
{
	 RTK_Read
};
//char Test[51]={'G','P','S','A'};
BOOL RTK_Read( u8 *StrAdd)
{
	RTK_GPS_ T1,T2,*T3;
	T2.Alt_M = 55;


	RTK_GPS_* RTK_GPS_Temp;
	RTK_GPS_Temp = (RTK_GPS_*)StrAdd;
	T1 = (RTK_GPS_)(*RTK_GPS_Temp);
	//RTK_GPS = T2;
	return True;
}




