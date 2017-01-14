#ifndef __GPS_H__
#define __GPS_H__
#include "stm32f4xx.h"
#include "Time.h"
#include "Common.h"
struct GPS_Data_
{
	u8 Count;
	float POS_X;
	float POS_Y;
};
struct GXGGA
{
	float UTC;          //UTC时间
	float LAT;          //纬度
	char  NorS;         //南北
	float LON;          //经度
	char WorE;          //东西
	u8 state;           //定位状态 0：未定位 1：无差分，SPS模式 2：带差分，SPS模式 3：PPS模式
	u8 SatNum;          //卫星数量
	float HDOP;         //水平经度衰减因子
	float AltSea;       //海平面高度
	float AltEarth;     //相对于海平面的地面高度
	u16 GPSBaseTime;    //上一次收到GPS差分站数据开始的计时，单位秒
	u16 GPSBase;        //差分站编号
	
};
struct GXGLL
{
	float UTC;          //UTC时间
	float LAT;          //纬度
	char  NorS;         //南北
	float LON;          //经度
	char WorE;          //东西
	u8 state;           //定位状态 A：有效 V：无效
};

struct GXRMC
{
	
};
struct GXVTG
{
	
};
extern struct GPS_
{
	struct GPS_Data_* Data;
	void (*Init)(void);
	void (*Updata)(void);
}GPS;

#endif

