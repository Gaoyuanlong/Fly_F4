#include "USART1.h"
void USART1_Init(u32 Bound);
void USART1_Send_Datas(void);

struct USART1_Cf_ USART1_Cf = 
{
	USART1_Init,
	USART1_Send_Datas
};

void USART1_Init(u32 Bound)
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

void USART1_Send_Str(u8 *data_to_send, u8 length)
{
	u8 i = 0;
	while(i < length)
	{
		USART_SendData(USART1,data_to_send[i]);	
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){} 
		i++;
	}

}

extern u8 KeyTest;
extern "C"{
void USART1_IRQHandler(void)                
{
	if(USART_GetITStatus(USART1,USART_IT_TXE) == SET)
	{
		USART_SendData(USART1,0);
	}

	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)
	{	
		u8 TMP = USART_ReceiveData(USART1);
		KeyTest = TMP ;
		//ANO_DT_Data_Receive_Prepare(TMP);
	} 	
}

}

u32 SendData[8] = {0};
void USART1_Send_Datas(void)
{
	static unsigned short int send_data[3][8] = { { 0 }, { 0 }, { 0 } };

	switch(KeyTest)
	{
		case '1':
			{
			send_data[0][0] = (unsigned short int)((RTK_XYZ_HP_Offset.Lon_M)*100);
			send_data[0][1] = (unsigned short int)(0);
			send_data[0][2] = (unsigned short int)(0);
			send_data[0][3] = (unsigned short int)(0);
			send_data[0][4] = (unsigned short int)(0);
			send_data[0][5] = (unsigned short int)(0);
			send_data[0][6] = (unsigned short int)(0);
			send_data[0][7] = (unsigned short int)(0);

			send_data[1][0] = (unsigned short int)((RTK_XYZ_HP_Offset.Lat_M)*100);
			send_data[1][1] = (unsigned short int)(0);
			send_data[1][2] = (unsigned short int)(0);
			send_data[1][3] = (unsigned short int)(0);
			send_data[1][4] = (unsigned short int)(0);
			send_data[1][5] = (unsigned short int)(0);
			send_data[1][6] = (unsigned short int)(0);
			send_data[1][7] = (unsigned short int)(0);

			send_data[2][0] = (unsigned short int)((RTK_XYZ_HP_Offset.Alt_M)*100);
			send_data[2][1] = (unsigned short int)(0);
			send_data[2][2] = (unsigned short int)(0);
			send_data[2][3] = (unsigned short int)(0);
			send_data[2][4] = (unsigned short int)(0);
			send_data[2][5] = (unsigned short int)(0);
			send_data[2][6] = (unsigned short int)(0);
			send_data[2][7] = (unsigned short int)(0);
			}break;
		case '2':
			{
			send_data[0][0] = (unsigned short int)(Position.Data->POS_X*100);
			send_data[0][1] = (unsigned short int)((RTK_XYZ_HP.Lon_M)*100 + 10);
			send_data[0][2] = (unsigned short int)(0);
			send_data[0][3] = (unsigned short int)(0);
			send_data[0][4] = (unsigned short int)(0);
			send_data[0][5] = (unsigned short int)(0);
			send_data[0][6] = (unsigned short int)(0);
			send_data[0][7] = (unsigned short int)(0);

			send_data[1][0] = (unsigned short int)(Position.Data->POS_Y*100);
			send_data[1][1] = (unsigned short int)((RTK_XYZ_HP.Lat_M)*100 + 10);
			send_data[1][2] = (unsigned short int)(0);
			send_data[1][3] = (unsigned short int)(0);
			send_data[1][4] = (unsigned short int)(0);
			send_data[1][5] = (unsigned short int)(0);
			send_data[1][6] = (unsigned short int)(0);
			send_data[1][7] = (unsigned short int)(0); 

			send_data[2][0] = (unsigned short int)(Position.Data->POS_Z*100);
			send_data[2][1] = (unsigned short int)((RTK_XYZ_HP.Alt_M)*100 + 10);
			send_data[2][2] = (unsigned short int)(0);
			send_data[2][3] = (unsigned short int)(0);
			send_data[2][4] = (unsigned short int)(0);
			send_data[2][5] = (unsigned short int)(0);
			send_data[2][6] = (unsigned short int)(0);
			send_data[2][7] = (unsigned short int)(0);
			}break;
		case '3':
			{
			send_data[0][0] = (unsigned short int)(Attitude.Angle->x + 90);
			send_data[0][1] = (unsigned short int)(0);
			send_data[0][2] = (unsigned short int)(0);
			send_data[0][3] = (unsigned short int)(0);
			send_data[0][4] = (unsigned short int)(0);
			send_data[0][5] = (unsigned short int)(0);
			send_data[0][6] = (unsigned short int)(0);
			send_data[0][7] = (unsigned short int)(0);

			send_data[1][0] = (unsigned short int)(Attitude.Angle->y + 90);
			send_data[1][1] = (unsigned short int)(0);
			send_data[1][2] = (unsigned short int)(0);
			send_data[1][3] = (unsigned short int)(0);
			send_data[1][4] = (unsigned short int)(0);
			send_data[1][5] = (unsigned short int)(0);
			send_data[1][6] = (unsigned short int)(0);
			send_data[1][7] = (unsigned short int)(0);

			send_data[2][0] = (unsigned short int)(Attitude.Angle->z + 180);
			send_data[2][1] = (unsigned short int)(RTK_XYZ_HP.Heading + 180 + 10);
			send_data[2][2] = (unsigned short int)(0);
			send_data[2][3] = (unsigned short int)(0);
			send_data[2][4] = (unsigned short int)(0);
			send_data[2][5] = (unsigned short int)(0);
			send_data[2][6] = (unsigned short int)(0);
			send_data[2][7] = (unsigned short int)(0);
			}break;
		case '4':
			{
			send_data[0][0] = (unsigned short int)(Position.Data->SPE_X*100);
			send_data[0][1] = (unsigned short int)(0);
			send_data[0][2] = (unsigned short int)(0);
			send_data[0][3] = (unsigned short int)(0);
			send_data[0][4] = (unsigned short int)(0);
			send_data[0][5] = (unsigned short int)(0);
			send_data[0][6] = (unsigned short int)(0);
			send_data[0][7] = (unsigned short int)(0);

			send_data[1][0] = (unsigned short int)(Position.Data->SPE_Y*100);
			send_data[1][1] = (unsigned short int)(0);
			send_data[1][2] = (unsigned short int)(0);
			send_data[1][3] = (unsigned short int)(0);
			send_data[1][4] = (unsigned short int)(0);
			send_data[1][5] = (unsigned short int)(0);
			send_data[1][6] = (unsigned short int)(0);
			send_data[1][7] = (unsigned short int)(0);

			send_data[2][0] = (unsigned short int)(Position.Data->SPE_Z*100);
			send_data[2][1] = (unsigned short int)(0);
			send_data[2][2] = (unsigned short int)(0);
			send_data[2][3] = (unsigned short int)(0);
			send_data[2][4] = (unsigned short int)(0);
			send_data[2][5] = (unsigned short int)(0);
			send_data[2][6] = (unsigned short int)(0);
			send_data[2][7] = (unsigned short int)(0);
			}break;
		case '5':
			{
			send_data[0][0] = (unsigned short int)(FlyControl.Para->POS_Inner_PID_z.Output);
			send_data[0][1] = (unsigned short int)(0);
			send_data[0][2] = (unsigned short int)(0);
			send_data[0][3] = (unsigned short int)(0);
			send_data[0][4] = (unsigned short int)(0);
			send_data[0][5] = (unsigned short int)(0);
			send_data[0][6] = (unsigned short int)(0);
			send_data[0][7] = (unsigned short int)(0);

			send_data[1][0] = (unsigned short int)(Position.Data->POS_Z*100);
			send_data[1][1] = (unsigned short int)(0);
			send_data[1][2] = (unsigned short int)(0);
			send_data[1][3] = (unsigned short int)(0);
			send_data[1][4] = (unsigned short int)(0);
			send_data[1][5] = (unsigned short int)(0);
			send_data[1][6] = (unsigned short int)(0);
			send_data[1][7] = (unsigned short int)(0);

			send_data[2][0] = (unsigned short int)(FlyControl.Para->POS_Outer_PID_z.Setpoint*100);
			send_data[2][1] = (unsigned short int)(0);
			send_data[2][2] = (unsigned short int)(0);
			send_data[2][3] = (unsigned short int)(0);
			send_data[2][4] = (unsigned short int)(0);
			send_data[2][5] = (unsigned short int)(0);
			send_data[2][6] = (unsigned short int)(0);
			send_data[2][7] = (unsigned short int)(0);
			}break;
		default: break;
	}

	USART_SendData(USART1, 'S');
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){} 
	USART_SendData(USART1, 'T');
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){} 
	for (int i = 0; i < 3; i++)
	for (int j = 0; j < 8; j++)
	{
		USART_SendData(USART1, (unsigned char)(send_data[i][j] & 0x00ff));
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){} 
		USART_SendData(USART1, (unsigned char)(send_data[i][j] >> 8u));
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){} 
	}
}

