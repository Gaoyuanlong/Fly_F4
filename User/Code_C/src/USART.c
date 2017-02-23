#include "USART.h"

BOOL IsTXBufIDEL = True;    //发送缓存空闲
BOOL IsRXBufIDEL = True;
u8 BUF_TMP[BUF_SIZE];

struct USART_Data_ USART_Data; 

void USART_Init_User(u32 Bound);
BOOL USART_SendChar(void);
BOOL USART_Receive(void);
BOOL USART_Request_TXBUF(void);
void USART_Free_RXBUF(void);


struct USART_ USART = 
{
	&USART_Data,
	USART_Init_User,
	USART_Request_TXBUF,
	USART_SendChar,
	USART_Receive,
	USART_Free_RXBUF
};


void USART_Init_User(u32 Bound)
{
	//GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	//USART3_TX   PB.10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//USART3_RX	  PB.11  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	//USART3 NVIC 
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	

	USART_InitStructure.USART_BaudRate = Bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART3, ENABLE);                   

}

//获取TXBUF使用权
BOOL USART_Request_TXBUF(void)
{
	if(IsTXBufIDEL == True)
	{
		BOOL_REVERSE(IsTXBufIDEL);
		return True;
	}
	else
		return False;
}
//发送数据包命令
BOOL USART_SendChar(void)
{
	if(IsTXBufIDEL == False)
	{
		USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
		return True;
	}
	else
	{
		return False;
	}
}
//接收数据
BOOL USART_Receive(void)
{
	if(IsRXBufIDEL == True)	return False;
	else	return True;
}
//释放RX数据包占用
void USART_Free_RXBUF(void)
{
	IsRXBufIDEL = True;
}

extern "C"{
void USART3_IRQHandler(void)                
{
	static u32 TimeStore;             //用于分帧
	static u8 USART_SendCnt = 0;      //记录发送数据的位置
	static u8 USART_ReceiveCnt = 0;   //记录接收数据的位置	
 
	//字节丢失中断，这个中断在开启接收中断时自动开启
	if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)  
	{  
		USART_ReceiveData(USART3);   
	}  
	
	if(USART_GetITStatus(USART3,USART_IT_TXE) == SET)
	{
		USART_SendData(USART3,USART_Data.TX_BUF[USART_SendCnt++]);
		
		if(USART_SendCnt == BUF_SIZE)
		{
			IsTXBufIDEL = True; //缓存空闲
			USART_SendCnt = 0;

			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		}

	}

	if(USART_GetITStatus(USART3,USART_IT_RXNE) == SET)
	{	
		u8 TMP = USART_ReceiveData(USART3);
		u32 TimeNow = SystemTime.Now_MS(); 
		if(TimeNow - TimeStore > 2)
		{
			USART_ReceiveCnt = 0;
			if(IsRXBufIDEL == True)
			{
				IsRXBufIDEL = False;
				for(u8 i = 0;i < BUF_SIZE;i++) 
					USART_Data.RX_BUF[i] = BUF_TMP[i];
			}
		}
		TimeStore = TimeNow;
		BUF_TMP[USART_ReceiveCnt++] = TMP;	
		if(USART_ReceiveCnt == BUF_SIZE) 	USART_ReceiveCnt--;
	} 	
}
}
