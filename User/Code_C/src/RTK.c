#include "RTK.h"

BOOL RTK_Read( u8 *StrAdd);

Filter_MidValue GPS_PX_Filter;
Filter_MidValue GPS_PY_Filter;
Filter_MidValue GPS_PZ_Filter;
Filter_MidValue GPS_VX_Filter;
Filter_MidValue GPS_VY_Filter;
Filter_MidValue GPS_VZ_Filter;
Filter_MidValue GPS_HE_Filter;
Filter_MidValue GPS_PI_Filter;
Filter_MidValue GPS_Lon_Filter;
Filter_MidValue GPS_Lat_Filter;
Filter_MidValue GPS_Alt_Filter;

RTK_XYZ_HP_ RTK_XYZ_HP;
RTK_XYZ_HP_ RTK_XYZ_HP_Offset;

RTK_OPS_ RTK_OPS=
{
	 RTK_Read
};

BOOL RTK_Read( u8 *StrAdd)
{
	static u8 OffsetGetFlag = 0;
	u8 Data_Str[RTK_GPS_SIZE] = {0}; 
	u32 Time_Now = 0;
	static u32 Time_Old = 0;
	#define DELAY_OFFSET_GET 2000
	//不能直接赋值 问题未找到原因
	for(u16 i = 0;i < RTK_GPS_SIZE;i++)
	{  
		Data_Str[i] = StrAdd[i];
	}
	
	RTK_XYZ_HP = *(RTK_XYZ_HP_*)Data_Str;
	
	RTK_XYZ_HP.PX = GPS_PX_Filter.MidValue(RTK_XYZ_HP.PX);
	RTK_XYZ_HP.PY = GPS_PY_Filter.MidValue(RTK_XYZ_HP.PY);
	RTK_XYZ_HP.PZ = GPS_PZ_Filter.MidValue(RTK_XYZ_HP.PZ); 
	RTK_XYZ_HP.VX = GPS_VX_Filter.MidValue(RTK_XYZ_HP.VX);
	RTK_XYZ_HP.VY = GPS_VY_Filter.MidValue(RTK_XYZ_HP.VY);
	RTK_XYZ_HP.VZ = GPS_VZ_Filter.MidValue(RTK_XYZ_HP.VZ);
	RTK_XYZ_HP.Heading = GPS_HE_Filter.MidValue(RTK_XYZ_HP.Heading);
	RTK_XYZ_HP.Pitch = GPS_PI_Filter.MidValue(RTK_XYZ_HP.Pitch);
	RTK_XYZ_HP.Lon_M = GPS_Lon_Filter.MidValue(RTK_XYZ_HP.Lon_M);
	RTK_XYZ_HP.Lat_M = GPS_Lat_Filter.MidValue(RTK_XYZ_HP.Lat_M);
	RTK_XYZ_HP.Alt_M = GPS_Alt_Filter.MidValue(RTK_XYZ_HP.Alt_M);	
	RTK_XYZ_HP.Quality = RTK_XYZ_HP.Quality;
	
	if(0 == OffsetGetFlag && 4 == RTK_XYZ_HP.Quality)
	{
		Time_Now = SystemTime.Now_MS();
		if(Time_Old == 0)
			Time_Old = Time_Now;
		if(DELAY_OFFSET_GET < (Time_Now - Time_Old) && 4 == RTK_XYZ_HP.Quality)
		{
			OffsetGetFlag = 1;
			RTK_XYZ_HP_Offset.PX = RTK_XYZ_HP.PX;
			RTK_XYZ_HP_Offset.PY = RTK_XYZ_HP.PY;
			RTK_XYZ_HP_Offset.PZ = RTK_XYZ_HP.PZ;
			RTK_XYZ_HP_Offset.Lon_M = RTK_XYZ_HP.Lon_M;
			RTK_XYZ_HP_Offset.Lat_M = RTK_XYZ_HP.Lat_M;
			RTK_XYZ_HP_Offset.Alt_M = RTK_XYZ_HP.Alt_M;
			RTK_XYZ_HP_Offset.Pitch =  -90;//-90~90转成0~180显示
			OffsetGetFlag = 1;
		}
	}
	return True;
}





