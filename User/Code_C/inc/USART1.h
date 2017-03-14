#ifndef __GPS_H__
#define __GPS_H__
#include "stm32f4xx.h"
#include "Time.h"
#include "Common.h"
#include "ANO_DT.h"

extern struct USART1_Cf_
{
	void (*Init)(u32 Bound);
	void (*USART1_Send_Datas)(void);
}USART1_Cf;


void USART1_Send_Str(u8 *data_to_send, u8 length);
	
#endif

