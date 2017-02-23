#include "RTK.h"

Filter_MidValue GPS_Lat_Filter;
Filter_MidValue GPS_Lon_Filter;
Filter_MidValue GPS_Alt_Filter;
Filter_MidValue GPS_TrackAngle_Filter;
Filter_MidValue GPS_Speed_Filter;

BOOL RTK_Updata( u8 *StrAdd);

RTK_GPS_ RTK_GPS;
struct Vector RTK_GPS_Speed; 

RTK_OPS_ RTK_OPS=
{
	 RTK_Updata
};

BOOL RTK_Updata( u8 *StrAdd)
{
	RTK_GPS_ RTK_GPS_Temp;
	u8 Data_Str[RTK_GPS_SIZE] = {0}; 
	static uint64_t Time_Old = 0;
	uint64_t Time_Enter = 0;
	
	Time_Enter= SystemTime.Now_US();
	
	for(int i = 0;i < RTK_GPS_SIZE;i++)
	{
		Data_Str[i] = StrAdd[i];
	}
	RTK_GPS_Temp = *(RTK_GPS_*)Data_Str;
	
	RTK_GPS_Temp.Lat_M = GPS_Lat_Filter.MidValue(RTK_GPS_Temp.Lat_M);
	RTK_GPS_Temp.Lon_M = GPS_Lon_Filter.MidValue(RTK_GPS_Temp.Lon_M);
	RTK_GPS_Temp.Alt_M = GPS_Alt_Filter.MidValue(RTK_GPS_Temp.Alt_M); 
	RTK_GPS_Temp.TrackAngle = GPS_TrackAngle_Filter.MidValue(RTK_GPS_Temp.TrackAngle);
	RTK_GPS_Temp.Speed_M = GPS_Speed_Filter.MidValue(RTK_GPS_Temp.Speed_M);
	
	RTK_GPS_Speed.x = (RTK_GPS_Temp.Lon_M - RTK_GPS.Lon_M)/(Time_Enter - Time_Old) * (double)1e6;
	RTK_GPS_Speed.y = (RTK_GPS_Temp.Lat_M - RTK_GPS.Lat_M)/(Time_Enter - Time_Old) * (double)1e6;
	RTK_GPS_Speed.z = (RTK_GPS_Temp.Alt_M - RTK_GPS.Alt_M)/(Time_Enter - Time_Old) * (double)1e6;
	
	/*
	*加入速度跳变判断
	*/
	
	RTK_GPS.Quality = RTK_GPS_Temp.Quality;
	RTK_GPS.Lat_M = RTK_GPS_Temp.Lat_M;
	RTK_GPS.Lon_M = RTK_GPS_Temp.Lon_M;
	RTK_GPS.Alt_M = RTK_GPS_Temp.Alt_M;
	RTK_GPS.TrackAngle = RTK_GPS_Temp.TrackAngle;
	RTK_GPS.Speed_M = RTK_GPS_Temp.Speed_M;
	
	Time_Old = Time_Enter;
	return True;
}





