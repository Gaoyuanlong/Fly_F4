#include "Filter.h"

Filter_2nd:: Filter_2nd(float Init_b0,float Init_b1,float Init_b2,float Init_a1,float Init_a2)
{
	b0 = Init_b0;
	b1 = Init_b1;
	b2 = Init_b2;
//a0 = 1;
	a1 = Init_a1;
	a2 = Init_a2;

	LastIn.x = 0;
	LastIn.y = 0;
	LastIn.z = 0;
	
	PreOut.x = 0;
	PreOut.y = 0;
	PreOut.z = 0;
	
	LastOut.x = 0;
	LastOut.y = 0;
	LastOut.z = 0;
}
/*
	二阶滤波器的基本差分方程
*/

Vector Filter_2nd:: LPF2ndFilter(Vector Input)
{
	Vector NewOut;

	NewOut.x = b0 * Input.x + b1 * LastIn.x + b2 * PreIn.x -  a1 * LastOut.x - a2 * PreOut.x ;
	NewOut.y = b0 * Input.y + b1 * LastIn.y + b2 * PreIn.y -  a1 * LastOut.y - a2 * PreOut.y;
	NewOut.z = b0 * Input.z + b1 * LastIn.z + b2 * PreIn.z -  a1 * LastOut.z - a2 * PreOut.z;

	PreIn = LastIn;
	LastIn = Input;
	
	PreOut = LastOut;
	LastOut = NewOut;
	
	return NewOut;
		
}

float Filter_2nd:: LPF2ndFilter(float Input)
{
	float NewOut;

	NewOut = b0 * Input + b1 * LastIn.x + b2 * PreIn.x -  a1 * LastOut.x - a2 * PreOut.x ;

	PreIn.x = LastIn.x;
	LastIn.x = Input;
	PreOut.x = LastOut.x;
	LastOut.x = NewOut;

	
	return NewOut;
		
}

/*
	冒泡排序
*/
float Filter_MidValue::Bubble(float Input[3])
{
	
	if(Input[0] < Input[1])
	{
		float buf = Input[0];
		Input[0] = Input[1];
		Input[1] = buf;
		
		if(Input[1] < Input[2])
		{
			float buf = Input[1];
			Input[1] = Input[2];
			Input[2] = buf;
			
			if(Input[0] < Input[1])
			{
				float buf = Input[0];
				Input[0] = Input[1];
				Input[1] = buf;
			}
		}
	}
	else
	{
		if(Input[1] < Input[2])
		{
			float buf = Input[1];
			Input[1] = Input[2];
			Input[2] = buf;
			
			if(Input[0] < Input[1])
			{
				float buf = Input[0];
				Input[0] = Input[1];
				Input[1] = buf;
			}
		}
	}
	
	return Input[1];
}
/*
	中值滤波
*/
Vector Filter_MidValue::MidValue(Vector Input)
{
	Vector Output;
	float Buf[3];
	
	Data[Cnt++] = Input;
	
	if(Cnt == 3) Cnt = 0;
	
	Buf[0] = Data[0].x;
	Buf[1] = Data[1].x;
	Buf[2] = Data[2].x;
	
	Output.x = Bubble(Buf);
	
	Buf[0] = Data[0].y;
	Buf[1] = Data[1].y;
	Buf[2] = Data[2].y;
	
	Output.y = Bubble(Buf);
	
	Buf[0] = Data[0].z;
	Buf[1] = Data[1].z;
	Buf[2] = Data[2].z;
	
	Output.z = Bubble(Buf);
	
	return Output;
}

float Filter_MidValue::MidValue(float Input)
{
	float Output;
	float Buf[3];
	
	Data1[Cnt++] = Input;
	
	if(Cnt == 3) Cnt = 0;
	
	Buf[0] = Data1[0];
	Buf[1] = Data1[1];
	Buf[2] = Data1[2];
	
	Output = Bubble(Buf);
	
	return Output;
}


float Filter_Fir::FirFilter(float Input)
{
	int i = 0;
	int j = 0;
	float Output = 0;
	Data[Cnt] = Input;

	for(i = 0;i < N;i++)
	{
		if(Cnt < i) j = N - i + Cnt; 
		else j = Cnt - i;
		Output += Data[j] * Para[i];
	}
	
	if(Cnt < N - 1) Cnt++;
	else
		Cnt = 0;	
	
	return Output; 
}

//互补滤波 -> 例如：加速度测量值和速度测量值共同 估计速度
//Input一次积分结果为输出数据
//Input受Measurement修正

float Filter_Balance::BalanceFilter(float Input,float Measurement,double dt)
{
	float Error,P_Error;
	
	Error = Measurement - Output;
	P_Error = Error * Kp;
	
	if(Ki != 0)
		SumError += Error * Ki;
	
	Input += P_Error;
	Input += SumError;
		
	Output += Input * dt;
	
	return Output;
}

float Filter_EKF::EKFFilter(float Model,float Input)
{
	X_k_k1 = Model;
	P_k_k1 = P_k_k + Q;
	X_k_k  = X_k_k1 + Kg * (Input - X_k_k1);
	Kg     = P_k_k1 / (P_k_k1 + R);
	P_k_k  = (1 - Kg) * P_k_k1;
	
	return X_k_k;
}

