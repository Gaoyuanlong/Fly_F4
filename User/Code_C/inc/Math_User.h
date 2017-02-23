#ifndef __MATH_USER_H__
#define __MATH_USER_H__
#include <stdint.h>
#include <stddef.h>
#include "arm_math.h"
#include "Common.h"
#define M_PI 3.141592653f
#define DEG_TO_RAD 0.01745329f
#define RAD_TO_DEG 57.29577951f

#define CONSTRAIN(X,UP,DOWN)  do{if(X > UP) X = UP;if(X < DOWN) X = DOWN;}while(0);
#define DEAD_ZONE(X,Limit) do{if(fabs(X) < Limit) X = 0;}while(0);


float Safe_Asin(float v);
//角度转弧度
float Radians(float deg);
//弧度转角度
float Degrees(float rad);

float To_180_degrees(float x);
class Quaternion
{
public:
	float q1, q2, q3, q4;

	Quaternion() 
	{
			q1 = 1; q2 = q3 = q4 = 0;
	}
};
extern struct Math_
{
	double (*Constrain)(double AimN, double Limit_Up,double Limit_Down);
	int (*Dead_Zone)(int AimN,int Limit);
	Vector (*Body_To_Earth)(Vector Body,float Pitch,float Roll,float Yaw);	
}Math;


#endif
