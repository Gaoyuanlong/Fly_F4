#include "GPS.h"
# 
#define GPS_BUF_SIZE 512
BOOL GPSReceived = False;
u8 GPS_BUF1[GPS_BUF_SIZE];
u8 GPS_BUF2[GPS_BUF_SIZE];
u8 *GPS_BUF_Point;

void GPS_Init(void);
void GPS_Updata(void);
struct GPS_Data_ GPS_Data;

struct GPS_ GPS = 
{
	&GPS_Data,
	GPS_Init,
	GPS_Updata
};

void USART_Init_GPS(u32 Bound)
{
	//GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//USART3_TX   PA9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART3_RX	  PA10  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART3 NVIC 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	

	USART_InitStructure.USART_BaudRate = Bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1, ENABLE);                   

}

void GPS_Init(void)
{
	USART_Init_GPS(9600);
}



void GPS_Updata(void)
{
	if(GPSReceived == True)
	{
		GPSReceived = False;

	}
}

extern "C"{
void USART1_IRQHandler(void)                
{
	static u32 TimeStore = 0;
	static u16 ReceiveCnt = 0;
	static BOOL BufOrder = False;
	if(USART_GetITStatus(USART1,USART_IT_TXE) == SET)
	{
		USART_SendData(USART1,0);
	}

	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)
	{	
		u8 TMP = USART_ReceiveData(USART1);
		if(SystemTime.Now_MS() - TimeStore > 2)
		{
			ReceiveCnt = 0;
			GPSReceived = True;
			BOOL_REVERSE(BufOrder);
		}

		if(BufOrder == True)	
		{
			GPS_BUF1[ReceiveCnt++] = TMP;
			GPS_BUF_Point = GPS_BUF2;
		}
		else 
		{
			GPS_BUF2[ReceiveCnt++] = TMP;
			GPS_BUF_Point = GPS_BUF1;
		}
		if(ReceiveCnt == GPS_BUF_SIZE) ReceiveCnt--;
		
		TimeStore = SystemTime.Now_MS();
	} 	
}

}

