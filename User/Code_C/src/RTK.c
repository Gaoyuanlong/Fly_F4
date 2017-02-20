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
	u8 Data_Str[RTK_GPS_SIZE] = {0}; 
	
	for(int i = 0;i < RTK_GPS_SIZE;i++)
	{
		Data_Str[i] = StrAdd[i];
	}
	
	RTK_GPS = *(RTK_GPS_*)Data_Str;
	return True;
}




