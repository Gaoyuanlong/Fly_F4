#include "SysState.h"

#define TIME_JUDGE_LOST  1000
#define TIME_BEFORE_LOCK 3000
#define TIME_TO_UNLOCK   3000

void State_Updata(u16 Time);

struct SystemState_ SystemState = 
{
	False,
	False,
	False,
	False,
	State_Updata
};

//失控判定
void RC_State(u16 Time)
{
	static int64_t LostTime = 0;
	
	if((SBUS.SW_POS_Judge(RC_AUX5) == Up) & (SBUS.SW_POS_Judge(RC_AUX6) == Up))
	{
		LostTime += Time;
		if(LostTime > TIME_JUDGE_LOST)
		{
			FlyControl.Para->IsLost = True;
			LostTime = TIME_JUDGE_LOST;
		}
	}
	else
	{
		FlyControl.Para->IsLost = False;
		LostTime = 0;	
	}
}

void Pow_State(u16 Time)
{
	BOOL IsAnyError = False;


	if(Power.Data->BAT_2S < 7.0f) SystemState.IsPowerALow = True;
	else SystemState.IsPowerALow = False;

#ifdef FOUR_ROTOR
	
#else
	if(Power.Data->BAT_12S < 44.0f) SystemState.IsPowerBLow = True;
	else SystemState.IsPowerBLow = False;	
	if(abs(Power.Data->POW_4V5 - 4.5f) > 0.5f)      IsAnyError = True;
	if(abs(Power.Data->POW_5V - 5.0f) > 0.5f)       IsAnyError = True;	
#endif

	if(abs(Power.Data->Main_3V3_MCU - 3.3f) > 0.5f) IsAnyError = True;
	if(abs(Power.Data->Main_3V3_SEN - 3.3f) > 0.5f) IsAnyError = True;
	if(abs(Power.Data->Main_5V - 5.0f) > 0.5f)      IsAnyError = True;	
	
	if(IsAnyError == True) SystemState.IsPowerError = True;
	else SystemState.IsPowerError = False;
	
}


void Sensor_State(u16 Time)
{
	SystemState.IsSensorReady = Position.IsReady;
}


void Controller_State(u16 Time)
{

	static u16 LockTime = 0;
	static u16 UnlockTime = 0;
	//解锁判定	 上锁判定
	if(FlyControl.Para->IsLock == True)
	{
		LockTime = 0;
		
		if(FlyControl.Para->IsLost == True) return;
		if(SystemState.IsSensorReady == False) return;
		if(SystemState.IsPowerError == True) return;
		if(SystemState.IsPowerALow == True) return;
		if(SystemState.IsPowerBLow == True) return;
		
		if((SBUS.SW_POS_Judge(RC_THROTTLE) == Down) & (SBUS.SW_POS_Judge(RC_YAW) == RIGHT))
		{
			UnlockTime += Time;
			if(UnlockTime > TIME_TO_UNLOCK)	
			{
				FlyControl.Para->IsLock = False;
			}
		}
		else	UnlockTime = 0;	
	}
	else
	{
		UnlockTime = 0;
		if(RC_THROTTLE < THROTTLE_MIN + 10)
			LockTime += Time;
		else
			LockTime = 0;
		if(LockTime > TIME_BEFORE_LOCK)
			FlyControl.Para->IsLock = True;
	}
}

void State_Show(u16 Time)
{
	static BOOL Sensor_State_Pre;
	if(SystemState.IsPowerALow == True || SystemState.IsPowerError == True) Buzzer.On(100);
	else 
		if((SystemState.IsSensorReady == True) & (Sensor_State_Pre == False)) 
		{
			Buzzer.On(0);
			SystemTime.WaitMS(1000);
		}
		else
		{
			Buzzer.Off();
		}
	Sensor_State_Pre = SystemState.IsSensorReady;
}


void State_Updata(u16 Time)
{
	RC_State(Time);
	Pow_State(Time);
	Sensor_State(Time);
	Controller_State(Time);
	State_Show(Time);
}


