#ifndef __USART_H__
#define __USART_H__
#include "stm32f4xx.h"
#include "Time.h"
#include "Common.h"
#include "LED.h"

#define BUF_SIZE 32

struct USART_Data_
{
	u8 TX_BUF[BUF_SIZE];
	u8 RX_BUF[BUF_SIZE];
};

extern struct USART_ 
{
	struct USART_Data_* Data;
	void (*Init)(u32 Bound);
	BOOL (*Request_TXBUF)(void);
	BOOL (*Send)(void);
	BOOL (*Receive)(void);
	void (*Free_RXBUF)(void);
}USART;


#endif
