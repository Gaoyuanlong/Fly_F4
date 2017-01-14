#include "Math_User.h"

double Constrain(double AimN, double Limit_Up,double Limit_Down);
int Dead_Zone(int AimN,int Limit);
Vector Body_To_Earth(Vector Body,float Pitch,float Roll);

struct Math_ Math =
{
	Constrain,
	Dead_Zone,
	Body_To_Earth
};


double Constrain(double AimN, double Limit_Up,double Limit_Down)
{
	if(AimN > Limit_Up) return Limit_Up;
	else
		if(AimN < Limit_Down) return Limit_Down;
		else return AimN;
}

int Dead_Zone(int AimN,int Limit)
{
	if(fabsf(AimN) < Limit) return 0;
	else return AimN;
}

/*
	坐标系关系说明：
	机体坐标系：以MPU6050芯片的坐标系一致  -> PWM捕获接口方向是机头方向
	            顺着坐标轴方向看去，顺时针方向为正方向
	            Pitch ： 绕坐标系Y轴运动		Vector Angle.y
	            Roll  ： 绕坐标系X轴运动		Vector Angle.x
							Yaw   ： 绕坐标系Z轴运动   Vector Angle.z
	参考坐标系：北东天坐标系  

*/

Vector Body_To_Earth(Vector Body,float Pitch,float Roll)
{
	Vector Earth;
	Pitch *= DEG_TO_RAD;
	Roll *= DEG_TO_RAD;
	float COS_Phi   = arm_cos_f32(Roll);
	float COS_Theta = arm_cos_f32(Pitch);
	float SIN_Phi   = arm_sin_f32(Roll);
	float SIN_Theta = arm_sin_f32(Pitch);
	
	Earth.x =  COS_Theta * Body.x + SIN_Phi * SIN_Theta * Body.y + COS_Phi * SIN_Theta * Body.z;
	Earth.y =                                 COS_Phi   * Body.y -             SIN_Phi * Body.z;
	Earth.z = -SIN_Theta * Body.x + SIN_Phi * COS_Theta * Body.y + COS_Phi * COS_Theta * Body.z;
	return Earth;
};

float To_180_degrees(float x)
{
	return (x>180?(x-360):(x<-180?(x+360):x));
}

//保证输入值是有效的
float Safe_Asin(float v)
{
    if (isnan(v)) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}
//角度转弧度
float Radians(float deg) 
	{
	return deg * DEG_TO_RAD;
}

//弧度转角度
float Degrees(float rad) 
	{
	return rad * RAD_TO_DEG;
}

