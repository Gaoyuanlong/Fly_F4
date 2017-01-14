#ifndef __PID_H__
#define __PID_H__

class PID
{
	public:
		PID(void);
		PID(float Init_Kp,float Init_Ki,float Init_Kd,float Init_I_Limit);
		
		double Cal_P(void);
		double Cal_I(int dt);
		void	 Rst(void);
		double Cal_D(int dt);	
		double Cal_PI_POS(int dt);		
		double Cal_PID_POS(int dt);	

		double Setpoint;
		double Feedback;
		double Output;
		double Kp;
		double Ki;
		double Kd;
		double SumError;
	private:

		double LastError;
		
		double I_Limit;
		double ValueTmp;

};

#endif
