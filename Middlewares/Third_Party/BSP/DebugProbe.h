/**
  ******************************************************************************
  * @file    DebugProbe.h
  * @author  Hossein Bagherzade
	* @email	 hossein.bagherzade@gmail.com	
  * @version V1.1.0
  * @date    10-Sep-2019
  * @brief   Header file for DebugProbe.c Library
  ******************************************************************************
	**/

#ifndef __DEBUG_PROBE_H
#define __DEBUG_PROBE_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_conf.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

int aPrintOutLog(const char* format, ... );
int aLeaveOutLog(const char* format, ... );	
	
void BSP_DebugProbe_Init(int baud);
	
UART_HandleTypeDef * BSP_DebugProbe_GetHandleTypeDef(void);
uint8_t* BSP_DebugProbe_GetRxDBuffer(void);

uint8_t BSP_DebugProbe_SendChar(uint8_t ch);
uint8_t BSP_DebugProbe_GetChar(void);
	
void BSP_DebugProbe_PutString(void *Str);
void BSP_DebugProbe_PutArray(void *str, uint16_t uiNum);

int sendchar(int ch);  
int getkey(void);  

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_PROBE_H */

