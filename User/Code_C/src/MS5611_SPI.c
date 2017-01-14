#include "MS5611_SPI.h"

#define MS5611_ADDRESS      0XEE
#define MS5611_RST          0X1E
#define MS5611_D1_OSR_256   0X40
#define MS5611_D1_OSR_512   0X42
#define MS5611_D1_OSR_1024  0X44
#define MS5611_D1_OSR_2048  0X46
#define MS5611_D1_OSR_4096  0X48
#define MS5611_D2_OSR_256   0X50
#define MS5611_D2_OSR_512   0X52
#define MS5611_D2_OSR_1024  0X54
#define MS5611_D2_OSR_2048  0X56
#define MS5611_D2_OSR_4096  0X58
#define MS5611_ADC_RD       0X00
#define MS5611_PROM_RD      0XA0
#define MS5611_PROM_CRC     0XAE

#define MS5611_IDEL         0X00
#define MS5611_TEMP_OK      0X01
#define MS5611_PRESS_OK     0X02
#define MS5611_DELAY        0X05  

#define CS1   GPIOB->BSRRL = GPIO_Pin_12
#define CS0   GPIOB->BSRRH = GPIO_Pin_12



static struct MS5611_Data_SPI_ MS5611_Data;
static int64_t dT;
static u16 PROM_C[7]; //用于存放PROM中的8组数据;
static u32 Press_ORI;
static u8 MS5611_State = MS5611_PRESS_OK;


/*
	气压计受温度影响极大，通电后需要一段时间稳定
	由于电路板发热，期间本身发热会导致温度升高
	读取频率保持固定，非常重要！！！

*/
static void MS561101_Init(void);
static void MS5611_Updata(void);


struct MS5611_SPI_ MS5611_SPI = 
{
	&MS5611_Data,
	MS561101_Init,
	MS5611_Updata
};


void MS5611_GPIO_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//CS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	CS1;

	//SPI	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //引脚初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);  //重映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);

}

void MS5611_SPI_Init(void)
{
	SPI_InitTypeDef    SPI_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE); 

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //全双工模式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;   //作为主机使用
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;   //数据长度8
	SPI_InitStructure.SPI_CPOL  = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;   //软件设置NSS功能
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //10M
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2,&SPI_InitStructure);
	SPI_Cmd(SPI2,ENABLE);

}

u8 MS5611_SPI_RW(u8 Data)
{
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}; 
  SPI_I2S_SendData(SPI2, Data);		
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){};
  return SPI_I2S_ReceiveData(SPI2);
}



static u8 MS5611_Reset(void)
{
	CS0;
	MS5611_SPI_RW(MS5611_RST);
	CS1;
	
	return 1;
}

//从PROM读取出厂校准数据
static void MS5611_PROM_Read(void)
{
	u16 Data1,Data2;
	u8 i;
	
	for(i = 0;i <= 6;i++)
	{
		CS0;
		MS5611_SPI_RW(MS5611_PROM_RD + i * 2);
		Data1 = MS5611_SPI_RW(0);
		Data2 = MS5611_SPI_RW(0);
		PROM_C[i]=(Data1 << 8) | Data2;
		CS1;
	}
	
}

static void MS5611_Send_Commond(u8 Commond)
{
	CS0;
	MS5611_SPI_RW(Commond);
	CS1;
}

static u32 MS5611_Read_3Byte(void)
{
	u32 Buf1,Buf2,Buf3; 
	CS0;
	MS5611_SPI_RW(0);
	
	Buf1 = MS5611_SPI_RW(0);
	Buf2 = MS5611_SPI_RW(0);
	Buf3 = MS5611_SPI_RW(0);
	CS1;
	
	return (Buf1 << 16) + (Buf2 << 8) + Buf3;
}

//读取数字温度
static BOOL MS5611_GetTemperature(void)
{
	static BOOL IsCommond = False;
	
	if(IsCommond == False) 
	{
		BOOL_REVERSE(IsCommond);
	
		MS5611_Send_Commond(MS5611_D2_OSR_4096);

		return False;
	}
	else
	{

		BOOL_REVERSE(IsCommond);		
		
		dT = MS5611_Read_3Byte();
		return True;
	}
}

//读取数字气压
static BOOL MS5611_GetPressure(void)
{
	static BOOL IsCommond = False;
	
	if(IsCommond == False) 
	{
		BOOL_REVERSE(IsCommond);
	
		MS5611_Send_Commond(MS5611_D1_OSR_4096);	
		
		return False;
	}
	else
	{
		BOOL_REVERSE(IsCommond);
		
		Press_ORI = MS5611_Read_3Byte();
				
		return True;	
	}	

}

static void MS561101_Init(void)
{
	MS5611_GPIO_Init();
	MS5611_SPI_Init();
	MS5611_Reset();
	SystemTime.WaitMS(100);
	MS5611_PROM_Read();
}
//采样频率25HZ 截止频率 2HZ
static Filter_2nd MS5611_Filter(0.046131802093312926,0.092263604186625853,0.046131802093312926,-1.3072850288493234 , 0.49181223722257528);
static Filter_MidValue MS5611_MID_Filter;

static void MS5611_Updata(void)
{
	
	switch (MS5611_State)
	{
		case MS5611_IDEL:
			if(MS5611_GetTemperature() == True) MS5611_State = MS5611_TEMP_OK;
			break;
		case MS5611_TEMP_OK:
			if(MS5611_GetPressure() == True)  MS5611_State = MS5611_PRESS_OK;
			break;
		default:
			break;
	}
	
	if(MS5611_State != MS5611_PRESS_OK) return;
	
	int64_t T2 = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;
	int64_t SENS,OFF;
	
	dT -= ((u32)PROM_C[5] << 8);
	MS5611_Data.Temperature = 2000 + (int32_t)(((int64_t)dT * PROM_C[6]) >> 23);
	OFF  = ((u32)PROM_C[2] << 16) + (((int64_t)PROM_C[4] * dT) >> 7);
	SENS = ((u32)PROM_C[1] << 15) + (((int64_t)PROM_C[3] * dT) >> 8);	

	if(MS5611_Data.Temperature < 2000)
	{

		T2 = (dT * dT) >> 31;				
		int64_t TMP = 5 * powl((int64_t)MS5611_Data.Temperature - 2000,2);

		OFF2  = TMP >> 1;
		SENS2 = TMP >> 2;
		
		if(MS5611_Data.Temperature < -1500)
		{
			int64_t TMP = powl((int64_t)MS5611_Data.Temperature + 1500,2);;
			OFF2 += 7 * TMP;
			SENS2 += 11 * TMP >> 1;
		}
	}	
	MS5611_Data.Temperature -= T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
	
	MS5611_Data.Pressure = (((Press_ORI * SENS) >> 21) - OFF) >> 15;	
	
	if(MS5611_Data.Pressure < 0) return;
	
	float Tmp = 0;
	
	Tmp = (1 - powf(MS5611_Data.Pressure / 101325.0f, 1.0f / 5.255f)) * 44330.0f;
	Tmp = MS5611_MID_Filter.MidValue(Tmp);	
	Tmp = MS5611_Filter.LPF2ndFilter(Tmp);
	MS5611_Data.Altitude = MS5611_Data.Altitude * 0.9f + Tmp * 0.1f;
	
	if(SystemTime.Now_MS() < 2000) 	MS5611_Data.Altitude_Ground = MS5611_Data.Altitude;
	
	MS5611_Data.Altitude -= MS5611_Data.Altitude_Ground;	
	
	MS5611_State = MS5611_IDEL;
	
}

