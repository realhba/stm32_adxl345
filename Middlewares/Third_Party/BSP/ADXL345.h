/**
  ******************************************************************************
  * @file    ADXL345.h
  * @author  Hossein Bagherzade(@realhba)
	* @email	 hossein.bagherzade@gmail.com
  * @version V1.1.3
  * @date    02-Jan-2018
  * @brief   Header file for ADXL345.c
  ******************************************************************************
	**/

#include "stm32f0xx.h"
#include "debugprobe.h"

#ifndef ADXL345_h
#define ADXL345_h

#define ACCL_DEBUG

#ifdef ACCL_DEBUG
#include "stdio.h"	
#define ACCL_LOG(...) 	aPrintOutLog( __VA_ARGS__) 
#else
#define ACCL_LOG(...) 
#endif

#define ACCL_SPI_INTERFACE								(1)

#define ACCL_SPI_CS_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOA_CLK_ENABLE()
#define ACCL_SPI_CS_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()
#define ACCL_SPI_SCK_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define ACCL_SPI_SCK_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOA_CLK_DISABLE()
#define ACCL_SPI_CS_LOW()      					 	HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET)
#define ACCL_SPI_CS_HIGH()      				 	HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET)
#define ACCL_SPI_SI_LOW()      					 	HAL_GPIO_WritePin(ADXL_MOSI_GPIO_Port, ADXL_MOSI_Pin, GPIO_PIN_RESET)
#define ACCL_SPI_SI_HIGH()      				 	HAL_GPIO_WritePin(ADXL_MOSI_GPIO_Port, ADXL_MOSI_Pin, GPIO_PIN_SET)
#define ACCL_SPI_CK_LOW()      					 	HAL_GPIO_WritePin(ADXL_CLK_GPIO_Port, ADXL_CLK_Pin, GPIO_PIN_RESET)
#define ACCL_SPI_CK_HIGH()      				 	HAL_GPIO_WritePin(ADXL_CLK_GPIO_Port, ADXL_CLK_Pin, GPIO_PIN_SET)
#define ACCL_SPI_SO_IS_HIGH()      			 	(HAL_GPIO_ReadPin(ADXL_MISO_GPIO_Port, ADXL_MISO_Pin) == GPIO_PIN_SET)

#define ACCL_SPI_FLAG_TIMEOUT          		((uint32_t) 200)			                               
#define WRITE_TIMEOUT_MS  			 				  (uint8_t)20 		// a write should only ever take 5 ms max

#define ACCL_CLK_DELAY(x)						 			for(int i = 0 ; i < (50 * x) ; i++) __NOP()
#define ACCL_SEQ_DELAY(x)						 			for(int i = 0 ; i < (20 * x) ; i++) __NOP()


/*************************** REGISTER MAP ***************************/
#define ADXL345_DEVID									0x00		// Device ID
#define ADXL345_RESERVED1							0x01		// Reserved. Do Not Access. 
#define ADXL345_THRESH_TAP						0x1D		// Tap Threshold. 
#define ADXL345_OFSX									0x1E		// X-Axis Offset. 
#define ADXL345_OFSY									0x1F		// Y-Axis Offset.
#define ADXL345_OFSZ									0x20		// Z- Axis Offset.
#define ADXL345_DUR										0x21		// Tap Duration.
#define ADXL345_LATENT								0x22		// Tap Latency.
#define ADXL345_WINDOW								0x23		// Tap Window.
#define ADXL345_THRESH_ACT						0x24		// Activity Threshold
#define ADXL345_THRESH_INACT					0x25		// Inactivity Threshold
#define ADXL345_TIME_INACT						0x26		// Inactivity Time
#define ADXL345_ACT_INACT_CTL					0x27		// Axis Enable Control for Activity and Inactivity Detection
#define ADXL345_THRESH_FF							0x28		// Free-Fall Threshold.
#define ADXL345_TIME_FF								0x29		// Free-Fall Time.
#define ADXL345_TAP_AXES							0x2A		// Axis Control for Tap/Double Tap.
#define ADXL345_ACT_TAP_STATUS				0x2B		// Source of Tap/Double Tap
#define ADXL345_BW_RATE								0x2C		// Data Rate and Power mode Control
#define ADXL345_POWER_CTL							0x2D		// Power-Saving Features Control
#define ADXL345_INT_ENABLE						0x2E		// Interrupt Enable Control
#define ADXL345_INT_MAP								0x2F		// Interrupt Mapping Control
#define ADXL345_INT_SOURCE						0x30		// Source of Interrupts
#define ADXL345_DATA_FORMAT						0x31		// Data Format Control
#define ADXL345_DATAX0								0x32		// X-Axis Data 0
#define ADXL345_DATAX1								0x33		// X-Axis Data 1
#define ADXL345_DATAY0								0x34		// Y-Axis Data 0
#define ADXL345_DATAY1								0x35		// Y-Axis Data 1
#define ADXL345_DATAZ0								0x36		// Z-Axis Data 0
#define ADXL345_DATAZ1								0x37		// Z-Axis Data 1
#define ADXL345_FIFO_CTL							0x38		// FIFO Control
#define ADXL345_FIFO_STATUS						0x39		// FIFO Status
					
#define ADXL345_BW_1600								0x0F		// 1111		IDD = 40uA
#define ADXL345_BW_800								0x0E		// 1110		IDD = 90uA
#define ADXL345_BW_400								0x0D		// 1101		IDD = 140uA
#define ADXL345_BW_200								0x0C		// 1100		IDD = 140uA
#define ADXL345_BW_100								0x0B		// 1011		IDD = 140uA 
#define ADXL345_BW_50									0x0A		// 1010		IDD = 140uA
#define ADXL345_BW_25									0x09		// 1001		IDD = 90uA
#define ADXL345_BW_12_5		  					0x08		// 1000		IDD = 60uA 
#define ADXL345_BW_6_25								0x07		// 0111		IDD = 50uA
#define ADXL345_BW_3_13								0x06		// 0110		IDD = 45uA
#define ADXL345_BW_1_56								0x05		// 0101		IDD = 40uA
#define ADXL345_BW_0_78								0x04		// 0100		IDD = 34uA
#define ADXL345_BW_0_39								0x03		// 0011		IDD = 23uA
#define ADXL345_BW_0_20								0x02		// 0010		IDD = 23uA
#define ADXL345_BW_0_10								0x01		// 0001		IDD = 23uA
#define ADXL345_BW_0_05								0x00		// 0000		IDD = 23uA


 /************************** INTERRUPT PINS **************************/
#define ADXL345_INT1_PIN							0x00		//INT1: 0
#define ADXL345_INT2_PIN							0x01		//INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define ADXL345_INT_DATA_READY_BIT		0x07
#define ADXL345_INT_SINGLE_TAP_BIT		0x06
#define ADXL345_INT_DOUBLE_TAP_BIT		0x05
#define ADXL345_INT_ACTIVITY_BIT			0x04
#define ADXL345_INT_INACTIVITY_BIT		0x03
#define ADXL345_INT_FREE_FALL_BIT			0x02
#define ADXL345_INT_WATERMARK_BIT			0x01
#define ADXL345_INT_OVERRUNY_BIT			0x00

#define ADXL345_DATA_READY						0x07
#define ADXL345_SINGLE_TAP						0x06
#define ADXL345_DOUBLE_TAP						0x05
#define ADXL345_ACTIVITY							0x04
#define ADXL345_INACTIVITY						0x03
#define ADXL345_FREE_FALL							0x02
#define ADXL345_WATERMARK							0x01
#define ADXL345_OVERRUNY							0x00


 /****************************** ERRORS ******************************/
#define ADXL345_OK										1		// No Error
#define ADXL345_ERROR									0		// Error Exists

#define ADXL345_NO_ERROR							0		// Initial State
#define ADXL345_READ_ERROR						1		// Accelerometer Reading Error
#define ADXL345_BAD_ARG								2		// Bad Argument

 /****************************** DEVICE ******************************/
#define ADXL345_DEVICE 										(0x53)    // Device Address for ADXL345
#define ADXL345_TO_READ 									(6)      	// Number of Bytes Read - Two Bytes Per Axis

 /****************************** SYSTEM ******************************/
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c %c%c%c%c "
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 
	
#define true 													1
#define false 												0	
	
typedef uint8_t bool;
typedef int8_t byte;

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

extern void ADXL345_AcclConfig(void);
extern void ADXL345_AcclProcess(void);
extern double ADXL345_AcclGetRoll(void);	
extern double ADXL345_AcclGetPitch(void);
extern uint8_t ADXL345_AcclGetActivity(void);

extern  void ADXL345_processInterfaceFunctionality(void);
extern  byte ADXL345_getInterfaceState(void);
extern  void ADXL345_setInterfaceState(byte st);

extern  void ADXL345_init(void);
extern  void ADXL345_powerOn(void);
extern  void ADXL345_readAccelAll(int *xyz);
extern	void ADXL345_readAccel(int* x, int* y, int* z);
extern	void ADXL345_get_Gxyz(double *xyz);
	
extern	void ADXL345_setTapThreshold(int tapThreshold);
extern	int  ADXL345_getTapThreshold(void);
extern	void ADXL345_setAxisGains(double *_gains);
extern	void ADXL345_getAxisGains(double *_gains);
extern	void ADXL345_setAxisOffset(int x, int y, int z);
extern	void ADXL345_getAxisOffset(int* x, int* y, int*z);
extern	void ADXL345_setTapDuration(int tapDuration);
extern	int  ADXL345_getTapDuration(void);
extern	void ADXL345_setDoubleTapLatency(int doubleTapLatency);
extern	int  ADXL345_getDoubleTapLatency(void);
extern	void ADXL345_setDoubleTapWindow(int doubleTapWindow);
extern	int  ADXL345_getDoubleTapWindow(void);
extern	void ADXL345_setActivityThreshold(int activityThreshold);
extern	int  ADXL345_getActivityThreshold(void);
extern	void ADXL345_setInactivityThreshold(int inactivityThreshold);
extern	int  ADXL345_getInactivityThreshold(void);
extern	void ADXL345_setTimeInactivity(int timeInactivity);
extern	int  ADXL345_getTimeInactivity(void);
extern	void ADXL345_setFreeFallThreshold(int freeFallthreshold);
extern	int  ADXL345_getFreeFallThreshold(void);
extern	void ADXL345_setFreeFallDuration(int freeFallDuration);
extern	int  ADXL345_getFreeFallDuration(void);
	
extern	bool ADXL345_isActivityXEnabled(void);
extern	bool ADXL345_isActivityYEnabled(void);
extern	bool ADXL345_isActivityZEnabled(void);
extern	bool ADXL345_isInactivityXEnabled(void);
extern	bool ADXL345_isInactivityYEnabled(void);
extern	bool ADXL345_isInactivityZEnabled(void);
extern	bool ADXL345_isActivityAc(void);
extern	bool ADXL345_isInactivityAc(void);
extern	void ADXL345_setActivityAc(bool state);
extern	void ADXL345_setInactivityAc(bool state);
	
extern	bool ADXL345_getSuppressBit(void);
extern	void ADXL345_setSuppressBit(bool state);
extern	bool ADXL345_isTapDetectionOnX(void);
extern	void ADXL345_setTapDetectionOnX(bool state);
extern	bool ADXL345_isTapDetectionOnY(void);
extern	void ADXL345_setTapDetectionOnY(bool state);
extern	bool ADXL345_isTapDetectionOnZ(void);
extern	void ADXL345_setTapDetectionOnZ(bool state);
extern	void ADXL345_setTapDetectionOnXYZ(bool stateX, bool stateY, bool stateZ);
	
extern	void ADXL345_setActivityX(bool state);
extern	void ADXL345_setActivityY(bool state);
extern	void ADXL345_setActivityZ(bool state);
extern	void ADXL345_setActivityXYZ(bool stateX, bool stateY, bool stateZ);
extern	void ADXL345_setInactivityX(bool state);
extern	void ADXL345_setInactivityY(bool state);
extern	void ADXL345_setInactivityZ(bool state);
extern	void ADXL345_setInactivityXYZ(bool stateX, bool stateY, bool stateZ);
	
extern	bool ADXL345_isActivitySourceOnX(void);
extern	bool ADXL345_isActivitySourceOnY(void);
extern	bool ADXL345_isActivitySourceOnZ(void);
extern	bool ADXL345_isTapSourceOnX(void);
extern	bool ADXL345_isTapSourceOnY(void);
extern	bool ADXL345_isTapSourceOnZ(void);
extern	bool ADXL345_isAsleep(void);
	
extern	bool ADXL345_isLowPower(void);
extern	void ADXL345_setLowPower(bool state);
extern	double ADXL345_getRate(void);
extern	void ADXL345_setRate(double rate);
extern	void ADXL345_set_bw(byte bw_code);
extern	byte ADXL345_get_bw_code(void);  
	
extern	bool ADXL345_triggered(byte interrupts, int mask);
	
extern	byte ADXL345_getInterruptSource(void);
extern	bool ADXL345_getInterruptSourceWithInt(byte interruptBit);
extern	bool ADXL345_getInterruptMapping(byte interruptBit);
extern	void ADXL345_setInterruptMapping(byte interruptBit, bool interruptPin);
extern	bool ADXL345_isInterruptEnabled(byte interruptBit);
extern	void ADXL345_setInterrupt(byte interruptBit, bool state);
extern	void ADXL345_setImportantInterruptMapping(int single_tap, int double_tap, int free_fall, int activity, int inactivity);
extern	void ADXL345_InactivityINT(bool status);
extern	void ADXL345_ActivityINT(bool status);
extern	void ADXL345_FreeFallINT(bool status);
extern	void ADXL345_doubleTapINT(bool status);
extern	void ADXL345_singleTapINT(bool status);
	
extern	void ADXL345_getRangeSetting(byte* rangeSetting);
extern	void ADXL345_setRangeSetting(int val);
extern	bool ADXL345_getSelfTestBit(void);
extern	void ADXL345_setSelfTestBit(bool selfTestBit);
extern	bool ADXL345_getSpiBit(void);
extern	void ADXL345_setSpiBit(bool spiBit);
extern	bool ADXL345_getInterruptLevelBit(void);
extern	void ADXL345_setInterruptLevelBit(bool interruptLevelBit);
extern	bool ADXL345_getFullResBit(void);
extern	void ADXL345_setFullResBit(bool fullResBit);
extern	bool ADXL345_getJustifyBit(void);
extern	void ADXL345_setJustifyBit(bool justifyBit);
extern	void ADXL345_printAllRegister(void);
	           
extern	void ADXL345_writeTo(byte address, byte val);
extern	void ADXL345_writeToI2C(byte address, byte val);
extern	void ADXL345_writeToSPI(byte address, byte val);
extern	void ADXL345_readFrom(byte address, int num, byte buff[]);
extern	void ADXL345_readFromI2C(byte address, int num, byte buff[]);
extern	void ADXL345_readFromSPI(byte address, int num, byte buff[]);
extern	void ADXL345_setRegisterBit(byte regAdress, int bitPos, bool state);
extern	bool ADXL345_getRegisterBit(byte regAdress, int bitPos);  

extern 	int ADXL345_constrain(int x, int a, int b);
extern void ADXL345_print_byte(byte val);

#endif
