/**
  ******************************************************************************
  * @file    DebugProbe.c
  * @author  Hossein Bagherzade(@realhba)
	* @email	 hossein.bagherzade@gmail.com
  * @version V1.1.0
  * @date    08-March-2018
  * @brief   Library for target-dependent low level functions 
  * All standard C I/O Functions, Real Link(vState), ISP And IAP Functions.
  ******************************************************************************
	**/	

#include "DebugProbe.h"		   
#include "stdarg.h"				   
#include "string.h"

extern UART_HandleTypeDef huart1;
uint8_t aRxBufferU1[1];


/*This Function outputs debug data to UART with Mutex Protection*/
//============================================
int aPrintOutLog(const char* format, ... )
//============================================	
{
	int rc;
	FILE *Fl;
	
  va_list args;
  va_start(args, format);
  rc = vfprintf(Fl, format, args);
  va_end(args);
  
   return rc;	
}

/*This Function Leaves out consoule Data*/
//============================================
int aLeaveOutLog(const char* format, ... )
//============================================	
{
  return 1;	
}

//=====================================
void BSP_DebugProbe_Init(int baud)
//=====================================
{	
  huart1.Instance = USART1;

  huart1.Init.BaudRate = baud;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

	HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBufferU1, 1);	
}

//============================================================
UART_HandleTypeDef * BSP_DebugProbe_GetHandleTypeDef(void)
//============================================================
{
	return (UART_HandleTypeDef * )&huart1;
}


//=========================================
uint8_t* BSP_DebugProbe_GetRxDBuffer(void)
//=========================================
{
	return aRxBufferU1;
}										   
//=============================================
uint8_t BSP_DebugProbe_SendChar(uint8_t ch)
//=============================================
{
	HAL_UART_Transmit(&huart1, &ch, 1, 100);
	return ch;
}

//========================================
uint8_t BSP_DebugProbe_GetChar(void)
//========================================
{
	uint8_t tmp = 0;
	HAL_UART_Receive(&huart1, &tmp, 1, 100);
	return(tmp);
}

//========================================
void BSP_DebugProbe_PutString(void *Str)
//========================================
{
	uint8_t *s = (uint8_t *) Str;
	while (*s!= NULL)
	{
		HAL_UART_Transmit(&huart1, s, 1, 100);
		s++;
	}
}


//========================================================
void BSP_DebugProbe_PutArray(void *str, uint16_t uiNum)
//========================================================	
{
		HAL_UART_Transmit(&huart1, (uint8_t *) str, uiNum, 100);
}


#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

//=====================
int  sendchar(int ch)
//=====================
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);	
 	return ch;
}

//=====================
int  getkey(void)
//=====================
{
 	unsigned char tmp = 0;
	HAL_UART_Receive(&huart1, &tmp, 1, 100);
 	return (int)tmp;
}

//===========================
int fputc(int ch, FILE *f) 
//===========================
{
  return (sendchar(ch));
}
//===========================
int fgetc(FILE *f)
//===========================
{
  return (sendchar(getkey()));
}
