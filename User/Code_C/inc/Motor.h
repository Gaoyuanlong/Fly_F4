#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stm32f4xx.h"
#include "Math_User.h"

#define PWM_LIMIT_U 1900
#define PWM_LIMIT_D 900
#define THROTTLE_HOLD 50
struct PWM_ 
{
	int PWM1;
	int PWM2;
	int PWM3;
	int PWM4;
	int PWM5;
	int PWM6;
};
 
extern struct Motor_
{
	struct PWM_* PWM;
	void (*Init)(void);
	void (*Output)(BOOL);
	void (*Stop)(void);
	void (*Hold)(void);
}Motor;

#endif
