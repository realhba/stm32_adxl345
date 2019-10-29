/**
  ******************************************************************************
  * @file    ADXL345.c
  * @author  Hossein Bagherzade(@realhba)
	* @email	 hossein.bagherzade@gmail.com
  * @version V1.1.3
  * @date    02-Jan-2018
  * @brief   Library for seting up ADXL345 3 axis accelerometer
  ******************************************************************************
	**/
	
#include "ADXL345.h"
#include "math.h"

static double gains[3];															// Counts to Gs
static byte _buff[6] ;															//	6 Bytes Buffer

static byte bInterfaceLiveState = ADXL345_OK;
static byte bInterfaceTotalState = ADXL345_OK;

//==============================================
HAL_StatusTypeDef ADXL345_SPI_Init(void)
//=============================================	
{
	ACCL_SPI_CS_GPIO_CLK_ENABLE();
	ACCL_SPI_SCK_GPIO_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Configure GPIO pins : ADXL_CLK_Pin */
  GPIO_InitStruct.Pin = ADXL_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADXL_CLK_GPIO_Port, &GPIO_InitStruct);	

  /*Configure GPIO pins : ADXL_MISO_Pin */
  GPIO_InitStruct.Pin = ADXL_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADXL_MISO_GPIO_Port, &GPIO_InitStruct);		

	/*Configure GPIO pins : ADXL_MOSI_Pin */
  GPIO_InitStruct.Pin = ADXL_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADXL_MOSI_GPIO_Port, &GPIO_InitStruct);	
	
	/*Configure GPIO pins : ADXL_CS_Pin */
  GPIO_InitStruct.Pin = ADXL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADXL_CS_GPIO_Port, &GPIO_InitStruct);	
	
	return HAL_OK;
}


//===========================================================
__STATIC_INLINE void ADXL345_SoftSPI_SendByte(uint8_t value)
//===========================================================	
{
  uint8_t clk = 0x08;
  
  while(clk > 0)
  {
     if((value & 0x80) != 0)
     {					
			 ACCL_SPI_SI_HIGH();
     }
     else
     {
       ACCL_SPI_SI_LOW();
     }
     
     ACCL_SPI_CK_LOW();
     ACCL_CLK_DELAY(1);				
     ACCL_SPI_CK_HIGH();
     ACCL_CLK_DELAY(1);
     
     value <<= 1;
     clk--;
  };
}

//=====================================================
__STATIC_INLINE uint8_t ADXL345_SoftSPI_RecvByte(void)
//=====================================================	
{
  uint8_t clk = 0x08;
  uint8_t temp = 0x00;
  
  while(clk > 0)
  {
    ACCL_SPI_CK_LOW();
    ACCL_CLK_DELAY(1);				
    ACCL_SPI_CK_HIGH();
    ACCL_CLK_DELAY(1);
    
    temp <<= 1;
    
    if(ACCL_SPI_SO_IS_HIGH())
    {
        temp |= 1;
    }
    
    clk--;
  };
  
  return temp;
}

//===========================
void ADXL345_init(void)
//===========================	
{
	ADXL345_SPI_Init();	
	ADXL345_powerOn();
	
	gains[0] = 0.00376390;		// Original gain 0.00376390 
	gains[1] = 0.00376009;		// Original gain 0.00376009
	gains[2] = 0.00349265;		// Original gain 0.00349265
}


//===========================
void ADXL345_powerOn(void)
//===========================	
{
	//ADXL345 TURN ON
	ADXL345_writeTo(ADXL345_POWER_CTL, 0);	// Wakeup     
	ADXL345_writeTo(ADXL345_POWER_CTL, 16);	// Auto_Sleep
	ADXL345_writeTo(ADXL345_POWER_CTL, 8);	// Measure
}


/*********************** READING ACCELERATION ***********************/
/*    Reads Acceleration into Three Variables:  x, y and z          */

//===================================
void ADXL345_readAccelAll(int *xyz)
//===================================	
{
	ADXL345_readAccel(xyz, xyz + 1, xyz + 2);
}


//===============================================
void ADXL345_readAccel(int *x, int *y, int *z)
//===============================================	
{
	ADXL345_readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff);	// Read Accel Data from ADXL345
	
	// Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
	*x = (((int)_buff[1]) << 8) | _buff[0];   
	*y = (((int)_buff[3]) << 8) | _buff[2];
	*z = (((int)_buff[5]) << 8) | _buff[4];
}

//==================================
void ADXL345_get_Gxyz(double *xyz)
//==================================	
{
	int i;
	int xyz_int[3];
	ADXL345_readAccelAll(xyz_int);
	for(i=0; i<3; i++){
		xyz[i] = xyz_int[i] * gains[i];
	}
}

/***************** WRITES VALUE TO ADDRESS REGISTER *****************/
//=============================================
void ADXL345_writeTo(byte address, byte val)
//=============================================	
{
#if (ACCL_SPI_INTERFACE == 1)	
	ADXL345_writeToSPI(address, val);
#else
	ADXL345_writeToI2C(address, val);
#endif	
}

/************************ READING NUM BYTES *************************/
/*    Reads Num Bytes. Starts from Address Reg to _buff Array        */
//===============================================================
void ADXL345_readFrom(byte address, int num, byte _buff[])
//===============================================================		
{	
#if (ACCL_SPI_INTERFACE == 1)	
	ADXL345_readFromSPI(address, num, _buff);	
#else
	ADXL345_readFromI2C(address, num, _buff);	
#endif	
}


/*************************** WRITE TO I2C ***************************/
/*      Start; Send Register Address; Send Value To Write; End      */
//==================================================
void ADXL345_writeToI2C(byte _address, byte _val)
//==================================================
{			
	if(HAL_I2C_Mem_Write(&hi2c1, (ADXL345_DEVICE << 1), (uint16_t) _address, (uint16_t) I2C_MEMADD_SIZE_8BIT, (uint8_t *)&_val, (uint16_t) 1, (uint32_t)100) != HAL_OK){
		bInterfaceLiveState = ADXL345_ERROR;
	}else{
		bInterfaceLiveState = ADXL345_OK;	
	}
}

/*************************** READ FROM I2C **************************/
/*                Start; Send Address To Read; End                  */
//===============================================================
void ADXL345_readFromI2C(byte address, int num, byte _buff[])
//===============================================================	
{				
	if(HAL_I2C_Mem_Read(&hi2c1, (ADXL345_DEVICE << 1), (uint16_t) address, (uint16_t) I2C_MEMADD_SIZE_8BIT, (uint8_t *)_buff, (uint16_t) num, (uint32_t)100) != HAL_OK){
		bInterfaceLiveState = ADXL345_ERROR;
	}else{
		bInterfaceLiveState = ADXL345_OK;	
	}
}


/************************** WRITE FROM SPI **************************/
/*         Point to Destination; Write Value; Turn Off              */
//===============================================================
void ADXL345_writeToSPI(byte __reg_address, byte __val)
//===============================================================
{	
	ACCL_SPI_CS_LOW();	
	ADXL345_SoftSPI_SendByte(__reg_address);
	ADXL345_SoftSPI_SendByte(__val);
	ACCL_SPI_CS_HIGH();			
}

/*************************** READ FROM SPI **************************/
/*                                                                  */
//===================================================================
void ADXL345_readFromSPI(byte __reg_address, int num, byte _buff[])
//===================================================================	
{
	// Read: Most Sig Bit of Reg Address Set
	char _address = 0x80 | __reg_address;
	// If Multi-Byte Read: Bit 6 Set 
	if(num > 1) {
		_address = _address | 0x40;
	}	
	
	ACCL_SPI_CS_LOW();	
	ADXL345_SoftSPI_SendByte(_address);		// Transfer Starting Reg Address To Be Read  
	for(int i=0; i<num; i++){
		_buff[i] = ADXL345_SoftSPI_RecvByte();
	}		
	ACCL_SPI_CS_HIGH();			
}


/*************************** RANGE SETTING **************************/
/*          ACCEPTABLE VALUES: 2g, 4g, 8g, 16g ~ GET & SET          */
//==================================================
void ADXL345_getRangeSetting(byte* rangeSetting)
//==================================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_DATA_FORMAT, 1, &_b);
	*rangeSetting = 0x03;
}

//=======================================
void ADXL345_setRangeSetting(int val)
//=======================================	
{
	byte _s;
	byte _b;
	
	switch (val) {
		case 2:  
			_s = 0x00; 
			break;
		case 4:  
			_s = 0x01; 
			break;
		case 8:  
			_s = 0x02; 
			break;
		case 16: 
			_s = 0x03; 
			break;
		default: 
			_s = 0x00;
	}
	ADXL345_readFrom(ADXL345_DATA_FORMAT, 1, &_b);
	_s |= (_b & 0xEC);
	ADXL345_writeTo(ADXL345_DATA_FORMAT, _s);
}

/*************************** SELF_TEST BIT **************************/
/*                            ~ GET & SET                           */
//================================
bool ADXL345_getSelfTestBit(void)
//================================
{
	return ADXL345_getRegisterBit(ADXL345_DATA_FORMAT, 7);
}

// If Set (1) Self-Test Applied. Electrostatic Force exerted on the sensor
//  causing a shift in the output data.
// If Set (0) Self-Test Disabled.
//==============================================
void ADXL345_setSelfTestBit(bool selfTestBit)
//==============================================
{
	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}

/*************************** SPI BIT STATE **************************/
/*                           ~ GET & SET                            */
//===========================
bool ADXL345_getSpiBit(void)
//===========================	
{
	return ADXL345_getRegisterBit(ADXL345_DATA_FORMAT, 6);
}

// If Set (1) Puts Device in 3-wire Mode
// If Set (0) Puts Device in 4-wire SPI Mode
//====================================
void ADXL345_setSpiBit(bool spiBit)
//====================================	
{
	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
}

/*********************** INT_INVERT BIT STATE ***********************/
/*                           ~ GET & SET                            */
//====================================
bool ADXL345_getInterruptLevelBit(void)
//====================================	
{
	return ADXL345_getRegisterBit(ADXL345_DATA_FORMAT, 5);
}

// If Set (0) Sets the Interrupts to Active HIGH
// If Set (1) Sets the Interrupts to Active LOW
//========================================================
void ADXL345_setInterruptLevelBit(bool interruptLevelBit)
//========================================================	
{
	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}

/************************* FULL_RES BIT STATE ***********************/
/*                           ~ GET & SET                            */
//=================================
bool ADXL345_getFullResBit(void)
//=================================	
{
	return ADXL345_getRegisterBit(ADXL345_DATA_FORMAT, 3);
}

// If Set (1) Device is in Full Resolution Mode: Output Resolution Increase with G Range
//  Set by the Range Bits to Maintain a 4mg/LSB Scale Factor
// If Set (0) Device is in 10-bit Mode: Range Bits Determine Maximum G Range
//  And Scale Factor
//================================================
void ADXL345_setFullResBit(bool fullResBit)
//================================================	
{
	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}

/*************************** JUSTIFY BIT STATE **************************/
/*                           ~ GET & SET                            */
//================================
bool ADXL345_getJustifyBit(void)
//================================
{
	return ADXL345_getRegisterBit(ADXL345_DATA_FORMAT, 2);
}

// If Set (1) Selects the Left Justified Mode
// If Set (0) Selects Right Justified Mode with Sign Extension
//=================================================
void ADXL345_setJustifyBit(bool justifyBit)
//================================================	
{
	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}

/*********************** THRESH_TAP BYTE VALUE **********************/
/*                          ~ SET & GET                             */
// Should Set Between 0 and 255
// Scale Factor is 62.5 mg/LSB
// A Value of 0 May Result in Undesirable Behavior
//================================================
void ADXL345_setTapThreshold(int tapThreshold)
//================================================	
{
	tapThreshold = ADXL345_constrain(tapThreshold,0,255);
	byte _b = (byte)(tapThreshold);
	ADXL345_writeTo(ADXL345_THRESH_TAP, _b);  
}

// Return Value Between 0 and 255
// Scale Factor is 62.5 mg/LSB
//================================
int ADXL345_getTapThreshold(void)
//================================
{
	byte _b;
	ADXL345_readFrom(ADXL345_THRESH_TAP, 1, &_b);  
	return ((int) (_b));
}

/****************** GAIN FOR EACH AXIS IN Gs / COUNT *****************/
/*                           ~ SET & GET                            */
//========================================
void ADXL345_setAxisGains(double *_gains)
//========================================
{
	int i;
	for(i = 0; i < 3; i++){
		gains[i] = _gains[i];
	}
}

//========================================
void ADXL345_getAxisGains(double *_gains)
//========================================	
{
	int i;
	for(i = 0; i < 3; i++){
		_gains[i] = gains[i];
	}
}

/********************* OFSX, OFSY and OFSZ BYTES ********************/
/*                           ~ SET & GET                            */
// OFSX, OFSY and OFSZ: User Offset Adjustments in Twos Complement Format
// Scale Factor of 15.6mg/LSB
//================================================
void ADXL345_setAxisOffset(int x, int y, int z)
//================================================	
{
	ADXL345_writeTo(ADXL345_OFSX, ((byte) (x)));  
	ADXL345_writeTo(ADXL345_OFSY, ((byte) (y)));  
	ADXL345_writeTo(ADXL345_OFSZ, ((byte) (z)));  
}

//================================================
void ADXL345_getAxisOffset(int* x, int* y, int*z)
//================================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_OFSX, 1, &_b);  
	*x = ((int) (_b));
	ADXL345_readFrom(ADXL345_OFSY, 1, &_b);  
	*y = ((int) (_b));
	ADXL345_readFrom(ADXL345_OFSZ, 1, &_b);  
	*z = ((int) (_b));
}

/****************************** DUR BYTE ****************************/
/*                           ~ SET & GET                            */
// DUR Byte: Contains an Unsigned Time Value Representing the Max Time 
//  that an Event must be Above the THRESH_TAP Threshold to qualify 
//  as a Tap Event
// The scale factor is 625µs/LSB
// Value of 0 Disables the Tap/Double Tap Funcitons. Max value is 255.
//=============================================
void ADXL345_setTapDuration(int tapDuration)
//=============================================	
{
	tapDuration = ADXL345_constrain(tapDuration,0,255);
	byte _b = ((byte) (tapDuration));
	ADXL345_writeTo(ADXL345_DUR, _b);  
}

//=======================================
int ADXL345_getTapDuration(void)
//=======================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_DUR, 1, &_b);  
	return ((int) (_b));
}

/************************** LATENT REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains Unsigned Time Value Representing the Wait Time from the Detection
//  of a Tap Event to the Start of the Time Window (defined by the Window 
//  Register) during which a possible Second Tap Even can be Detected.
// Scale Factor is 1.25ms/LSB. 
// A Value of 0 Disables the Double Tap Function.
// It Accepts a Maximum Value of 255.
//=======================================================
void ADXL345_setDoubleTapLatency(int doubleTapLatency)
//=======================================================	
{
	byte _b = (byte) (doubleTapLatency);
	ADXL345_writeTo(ADXL345_LATENT, _b);  
}

//=======================================
int ADXL345_getDoubleTapLatency(void)
//=======================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_LATENT, 1, &_b);  
	return ((int) (_b));
}

/************************** WINDOW REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains an Unsigned Time Value Representing the Amount of Time 
//  After the Expiration of the Latency Time (determined by Latent register)
//  During which a Second Valid Tape can Begin. 
// Scale Factor is 1.25ms/LSB. 
// Value of 0 Disables the Double Tap Function. 
// It Accepts a Maximum Value of 255.
//=======================================================
void ADXL345_setDoubleTapWindow(int doubleTapWindow)
//=======================================================	
{
	doubleTapWindow = ADXL345_constrain(doubleTapWindow,0,255);
	byte _b = (byte) (doubleTapWindow);
	ADXL345_writeTo(ADXL345_WINDOW, _b);  
}

//====================================
int ADXL345_getDoubleTapWindow(void)
//====================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_WINDOW, 1, &_b);  
	return ((int) (_b));
}

/*********************** THRESH_ACT REGISTER ************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Activity.
// Data Format is Unsigned, so the Magnitude of the Activity Event is Compared 
//  with the Value is Compared with the Value in the THRESH_ACT Register. 
// The Scale Factor is 62.5mg/LSB. 
// Value of 0 may Result in Undesirable Behavior if the Activity Interrupt Enabled. 
// It Accepts a Maximum Value of 255.
//=======================================================
void ADXL345_setActivityThreshold(int activityThreshold)
//=======================================================	
{
	activityThreshold = ADXL345_constrain(activityThreshold,0,255);
	byte _b =(byte) (activityThreshold);
	ADXL345_writeTo(ADXL345_THRESH_ACT, _b);  
}

// Gets the THRESH_ACT byte
//======================================
int ADXL345_getActivityThreshold(void)
//======================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_THRESH_ACT, 1, &_b);  
	return ((int) (_b));
}

/********************** THRESH_INACT REGISTER ***********************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Inactivity.
// The Data Format is Unsigned, so the Magnitude of the INactivity Event is 
//  Compared with the value in the THRESH_INACT Register. 
// Scale Factor is 62.5mg/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Inactivity Interrupt Enabled. 
// It Accepts a Maximum Value of 255.
//===========================================================
void ADXL345_setInactivityThreshold(int inactivityThreshold)
//===========================================================	
{
	inactivityThreshold = ADXL345_constrain(inactivityThreshold,0,255);
	byte _b = (byte) (inactivityThreshold);
	ADXL345_writeTo(ADXL345_THRESH_INACT, _b);  
}

//=========================================
int ADXL345_getInactivityThreshold(void)
//=========================================
{
	byte _b;
	ADXL345_readFrom(ADXL345_THRESH_INACT, 1, &_b);  
	return ((int) (_b));
}

/*********************** TIME_INACT RESIGER *************************/
/*                          ~ SET & GET                             */
// Contains an Unsigned Time Value Representing the Amount of Time that
//  Acceleration must be Less Than the Value in the THRESH_INACT Register
//  for Inactivity to be Declared. 
// Uses Filtered Output Data* unlike other Interrupt Functions
// Scale Factor is 1sec/LSB. 
// Value Must Be Between 0 and 255. 
//=================================================
void ADXL345_setTimeInactivity(int timeInactivity)
//=================================================	
{
	timeInactivity = ADXL345_constrain(timeInactivity,0,255);
	byte _b = (byte) (timeInactivity);
	ADXL345_writeTo(ADXL345_TIME_INACT, _b);  
}

//====================================
int ADXL345_getTimeInactivity(void)
//====================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_TIME_INACT, 1, &_b);  
	return ((int) (_b));
}

/*********************** THRESH_FF Register *************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value, in Unsigned Format, for Free-Fall Detection
// The Acceleration on all Axes is Compared with the Value in THRES_FF to
//  Determine if a Free-Fall Event Occurred. 
// Scale Factor is 62.5mg/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Free-Fall interrupt Enabled.
// Accepts a Maximum Value of 255.
//=======================================================
void ADXL345_setFreeFallThreshold(int freeFallThreshold)
//=======================================================
{
	freeFallThreshold = ADXL345_constrain(freeFallThreshold,0,255);
	byte _b = (byte) (freeFallThreshold);
	ADXL345_writeTo(ADXL345_THRESH_FF, _b);  
}

//=======================================
int ADXL345_getFreeFallThreshold(void)
//=======================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_THRESH_FF, 1, &_b);  
	return ((int) (_b));
}

/************************ TIME_FF Register **************************/
/*                          ~ SET & GET                             */
// Stores an Unsigned Time Value Representing the Minimum Time that the Value 
//  of all Axes must be Less Than THRES_FF to Generate a Free-Fall Interrupt.
// Scale Factor is 5ms/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Free-Fall Interrupt Enabled.
// Accepts a Maximum Value of 255.
//========================================================
void ADXL345_setFreeFallDuration(int freeFallDuration)
//========================================================	
{
	freeFallDuration = ADXL345_constrain(freeFallDuration,0,255);  
	byte _b = (byte) (freeFallDuration);
	ADXL345_writeTo(ADXL345_TIME_FF, _b);  
}

//=======================================
int ADXL345_getFreeFallDuration(void)
//=======================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_TIME_FF, 1, &_b);  
	return ((int) (_b));
}

/************************** ACTIVITY BITS ***************************/
/*                                                                  */

//=======================================
bool ADXL345_isActivityXEnabled(void)
//=======================================	
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 6); 
}

//=======================================
bool ADXL345_isActivityYEnabled(void)
//=======================================	
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 5); 
}

//=======================================
bool ADXL345_isActivityZEnabled(void)
//=======================================	
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 4); 
}

//=======================================
bool ADXL345_isInactivityXEnabled(void)
//=======================================	
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 2); 
}

//=======================================
bool ADXL345_isInactivityYEnabled(void)
//=======================================	
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 1); 
}

//=======================================
bool ADXL345_isInactivityZEnabled(void)
//=======================================	
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 0); 
}

//=======================================
void ADXL345_setActivityX(bool state)
//=======================================	
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state); 
}

//=======================================
void ADXL345_setActivityY(bool state)
//=======================================	
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state); 
}

//=======================================
void ADXL345_setActivityZ(bool state)
//=======================================	
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state); 
}

//=================================================================
void ADXL345_setActivityXYZ(bool stateX, bool stateY, bool stateZ)
//=================================================================	
{
	ADXL345_setActivityX(stateX);
	ADXL345_setActivityY(stateY);
	ADXL345_setActivityZ(stateZ);
}

//========================================
void ADXL345_setInactivityX(bool state)
//========================================	
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state); 
}

//========================================
void ADXL345_setInactivityY(bool state)
//========================================	
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state); 
}

//========================================
void ADXL345_setInactivityZ(bool state)
//========================================	
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state); 
}

//====================================================================
void ADXL345_setInactivityXYZ(bool stateX, bool stateY, bool stateZ)
//====================================================================	
{
	ADXL345_setInactivityX(stateX);
	ADXL345_setInactivityY(stateY);
	ADXL345_setInactivityZ(stateZ);
}

//=============================
bool ADXL345_isActivityAc(void)
//=============================	
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 7); 
}

//=============================
bool ADXL345_isInactivityAc(void)
//=============================	
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 3); 
}

//============================================
void ADXL345_setActivityAc(bool state)
//============================================	
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state); 
}

//============================================
void ADXL345_setInactivityAc(bool state) 
//============================================	
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 3, state); 
}

/************************* SUPPRESS BITS ****************************/
/*                                                                  */

//=================================
bool ADXL345_getSuppressBit(void)
//=================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 3); 
}

//============================================
void ADXL345_setSuppressBit(bool state)
//============================================	
{  
	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 3, state); 
}

/**************************** TAP BITS ******************************/
/*                                                                  */

//============================================
bool ADXL345_isTapDetectionOnX(void)
//============================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 2); 
}

//============================================
void ADXL345_setTapDetectionOnX(bool state)
//============================================	
{  
	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 2, state); 
}

//=====================================
bool ADXL345_isTapDetectionOnY(void)
//=====================================
{ 
	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 1); 
}

//============================================
void ADXL345_setTapDetectionOnY(bool state)
//============================================	
{  
	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 1, state); 
}

//============================================
bool ADXL345_isTapDetectionOnZ(void)
//============================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 0); 
}

//============================================
void ADXL345_setTapDetectionOnZ(bool state)
//============================================	
{  
	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 0, state); 
}

//=============================================================================
void ADXL345_setTapDetectionOnXYZ(bool stateX, bool stateY, bool stateZ)
//=============================================================================
{
	ADXL345_setTapDetectionOnX(stateX);
	ADXL345_setTapDetectionOnY(stateY);
	ADXL345_setTapDetectionOnZ(stateZ);
}

//=====================================
bool ADXL345_isActivitySourceOnX(void)
//=====================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 6); 
}

//=====================================
bool ADXL345_isActivitySourceOnY(void)
//=====================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 5); 
}

//======================================
bool ADXL345_isActivitySourceOnZ(void)
//======================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 4); 
}

//=================================
bool ADXL345_isTapSourceOnX(void)
//=================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 2); 
}

//=================================
bool ADXL345_isTapSourceOnY(void)
//=================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 1); 
}

//=================================
bool ADXL345_isTapSourceOnZ(void)
//=================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 0); 
}

/*************************** ASLEEP BIT *****************************/
/*                                                                  */
//=================================
bool ADXL345_isAsleep(void)
//=================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 3); 
}

/************************** LOW POWER BIT ***************************/
/*                                                                  */
//=================================
bool ADXL345_isLowPower(void)
//=================================	
{ 
	return ADXL345_getRegisterBit(ADXL345_BW_RATE, 4); 
}

//====================================
void ADXL345_setLowPower(bool state)
//====================================	
{  
	ADXL345_setRegisterBit(ADXL345_BW_RATE, 4, state); 
}

/*************************** RATE BITS ******************************/
/*                                                                  */

//=================================
double ADXL345_getRate(void)
//=================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_BW_RATE, 1, &_b);
	_b &= 0x0F;
	return (pow(2,((int) _b)-6)) * 6.25;
}

//=================================
void ADXL345_setRate(double rate)
//=================================	
{
	byte _b,_s;
	int v = (int) (rate / 6.25);
	int r = 0;
	while (v >>= 1)
	{
		r++;
	}
	if (r <= 9) { 
		ADXL345_readFrom(ADXL345_BW_RATE, 1, &_b);
		_s = (byte) (r + 6) | (_b & 0xF0);
		ADXL345_writeTo(ADXL345_BW_RATE, _s);
	}
}

/*************************** BANDWIDTH ******************************/
/*                          ~ SET & GET                             */
//=================================
void ADXL345_set_bw(byte bw_code)
//=================================	
{
	if((bw_code >= ADXL345_BW_0_05) && (bw_code <= ADXL345_BW_1600)){
		ADXL345_writeTo(ADXL345_BW_RATE, bw_code);		
	}
}

//==============================
byte ADXL345_get_bw_code(void)
//==============================
{
	byte bw_code;
	ADXL345_readFrom(ADXL345_BW_RATE, 1, &bw_code);
	return bw_code;
}




/************************* TRIGGER CHECK  ***************************/
/*                                                                  */
// Check if Action was Triggered in Interrupts
// Example triggered(interrupts, ADXL345_SINGLE_TAP);
//================================================
bool ADXL345_triggered(byte interrupts, int mask)
//================================================	
{
	return ((interrupts >> mask) & 1);
}

/*
 ADXL345_DATA_READY
 ADXL345_SINGLE_TAP
 ADXL345_DOUBLE_TAP
 ADXL345_ACTIVITY
 ADXL345_INACTIVITY
 ADXL345_FREE_FALL
 ADXL345_WATERMARK
 ADXL345_OVERRUNY
 */

//======================================
byte ADXL345_getInterruptSource(void)
//======================================	
{
	byte _b;
	ADXL345_readFrom(ADXL345_INT_SOURCE, 1, &_b);
	return _b;
}

//========================================================
bool ADXL345_getInterruptSourceWithInt(byte interruptBit)
//========================================================
{
	return ADXL345_getRegisterBit(ADXL345_INT_SOURCE,interruptBit);
}

//===================================================
bool ADXL345_getInterruptMapping(byte interruptBit)
//===================================================	
{
	return ADXL345_getRegisterBit(ADXL345_INT_MAP,interruptBit);
}

/*********************** INTERRUPT MAPPING **************************/
/*         Set the Mapping of an Interrupt to pin1 or pin2          */
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
//=======================================================================
void ADXL345_setInterruptMapping(byte interruptBit, bool interruptPin)
//=======================================================================	
{
	ADXL345_setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}

//======================================================================================================================
void ADXL345_setImportantInterruptMapping(int single_tap, int double_tap, int free_fall, int activity, int inactivity)
//======================================================================================================================	
{
	if(single_tap == 1) {
		ADXL345_setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );}
	else if(single_tap == 2) {
		ADXL345_setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT2_PIN );}

	if(double_tap == 1) {
		ADXL345_setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );}
	else if(double_tap == 2) {
		ADXL345_setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT2_PIN );}

	if(free_fall == 1) {
		ADXL345_setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,   ADXL345_INT1_PIN );}
	else if(free_fall == 2) {
		ADXL345_setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,   ADXL345_INT2_PIN );}

	if(activity == 1) {
		ADXL345_setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT1_PIN );}
	else if(activity == 2) {
		ADXL345_setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT2_PIN );}

	if(inactivity == 1) {
		ADXL345_setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );}
	else if(inactivity == 2) {
		ADXL345_setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT2_PIN );}
}

//========================================================
bool ADXL345_isInterruptEnabled(byte interruptBit)
//========================================================
{
	return ADXL345_getRegisterBit(ADXL345_INT_ENABLE,interruptBit);
}

//========================================================
void ADXL345_setInterrupt(byte interruptBit, bool state)
//========================================================	
{
	ADXL345_setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

//======================================
void ADXL345_singleTapINT(bool status)
//======================================	
{
	if(status) {
		ADXL345_setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
	}
	else {
		ADXL345_setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 0);
	}
}

//======================================
void ADXL345_doubleTapINT(bool status)
//======================================	
{
	if(status) {
		ADXL345_setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
	}
	else {
		ADXL345_setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 0);		
	}	
}

//======================================
void ADXL345_FreeFallINT(bool status)
//======================================	
{
	if(status) {
		ADXL345_setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
	}
	else {
		ADXL345_setInterrupt( ADXL345_INT_FREE_FALL_BIT,  0);
	}	
}

//======================================
void ADXL345_ActivityINT(bool status)
//======================================
{
	if(status) {
		ADXL345_setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
	}
	else {
		ADXL345_setInterrupt( ADXL345_INT_ACTIVITY_BIT,   0);
	}
}

//=======================================
void ADXL345_InactivityINT(bool status)
//=======================================
{
	if(status) {
		ADXL345_setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
	}
	else {
		ADXL345_setInterrupt( ADXL345_INT_INACTIVITY_BIT, 0);
	}
}

//===================================================================
void ADXL345_setRegisterBit(byte regAdress, int bitPos, bool state)
//===================================================================	
{
	byte _b;
	ADXL345_readFrom(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.  
	} 
	else {
		_b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
	}
	ADXL345_writeTo(regAdress, _b);  
}

//=======================================================
bool ADXL345_getRegisterBit(byte regAdress, int bitPos)
//=======================================================	
{
	byte _b;
	ADXL345_readFrom(regAdress, 1, &_b);
	return ((_b >> bitPos) & 1);
}

//===========================================
int ADXL345_constrain(int x, int a, int b)
//===========================================	
{
	if(x > b){
		return b;
	}
	if(x < a){
		return a;
	}
	
	return x;
}





/********************************************************************/
/*                                                                  */
// Print Register Values to Serial Output =
// Can be used to Manually Check the Current Configuration of Device
//==================================
void ADXL345_printAllRegister(void)
//==================================	
{
	byte _b;
	ADXL345_readFrom(0x00, 1, &_b);
	ACCL_LOG("0x00: "); 
	ACCL_LOG(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(_b));
	ACCL_LOG("\r\n"); 

	for (int i=29;i<=57;i++)
	{
		ADXL345_readFrom(i, 1, &_b);
		ACCL_LOG("0x0X: ", i); 
		ACCL_LOG(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(_b));
		ACCL_LOG("\r\n"); 
	}
}


/********************************************************************/
/*                                                                  */
//check hardware interface periodically and reInit in case of any errors.
// this is useful for run time diagnostic 
//===============================================
void ADXL345_processInterfaceFunctionality(void)
//===============================================
{
	static uint32_t uwInTick = 0;
	static uint8_t errCount = 0;
	uint8_t buff[1] = {0};
	
	if(HAL_GetTick() - uwInTick > 1000)
	{
			
		ADXL345_readFrom(ADXL345_DEVID, 1, (byte *)buff);	
		if(buff[0] == 229) bInterfaceLiveState = ADXL345_OK;
		if(buff[0] != 229) bInterfaceLiveState = ADXL345_ERROR;
		
		if(bInterfaceLiveState == ADXL345_ERROR){
			errCount++;
		}else{
			errCount = 0;
		}
		
		if(errCount > 10)
		{
			ACCL_LOG("ACCL_LOG | Hardware Interface Error!\r\n");
			ADXL345_setInterfaceState(0);
			
			//Try to Recover bus.
			ADXL345_init();
			ADXL345_powerOn();
			
			//Recover interface state for normal operation
			if(bInterfaceLiveState == ADXL345_OK){
				ACCL_LOG("ACCL_LOG | Hardware Interface Recovered!\r\n");
				ADXL345_setInterfaceState(1);
				errCount = 0;				
			}
		}
		
		uwInTick = HAL_GetTick();
	}
}


//====================================
byte ADXL345_getInterfaceState(void)
//====================================
{
	return bInterfaceTotalState;
}

//=====================================
void ADXL345_setInterfaceState(byte st)
//=====================================
{
		bInterfaceTotalState = st;
}

static uint8_t uAcclActiveState = 0;
float alpha = 0.5;
float fXg = 0, fYg = 0, fZg = 0, roll[5], pitch[5];
double rollAvg = 0, pitchAvg = 0;
double rollAvgOld = 0, pitchAvgOld= 0;
uint8_t avgIndex = 0, acclDetection = 0;
uint32_t uwDeactiveTM = 0;

#define PROCESS_NUM															(5)
#define DETECTION_THRESHOLD											(2.5f)
#define DEACTIVE_TIME														(60)

#include "math.h"

/********************************************************************/
/*                                                                  */
//check hardware interface periodically and reInit in case of any errors.
// this is useful for run time diagnostic 

//=============================
void ADXL345_AcclConfig(void)
//=============================
{
	ADXL345_init();
	
  ADXL345_powerOn();                     											// Power on the ADXL345
	
  ADXL345_setRangeSetting(16);   															// Give the range settings
																															// Accepted values are 2g, 4g, 8g or 16g
																															// Higher Values = Wider Measurement Range
																															// Lower Values = Greater Sensitivity
}

//=============================
void ADXL345_AcclProcess(void)
//=============================
{
	int x,y,z; 

		if(ADXL345_getInterfaceState())
		{
			// Accelerometer Readings 
			ADXL345_readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z
			//Low Pass Filter
			fXg = x * alpha + (fXg * (1.0 - alpha));
			fYg = y * alpha + (fYg * (1.0 - alpha));
			fZg = z * alpha + (fZg * (1.0 - alpha));
	
			//Roll & Pitch Equations
			roll[avgIndex]  = (atan2(-fYg, fZg) * 180.0f) / 3.1415f;
			pitch[avgIndex] = (atan2(fXg, sqrt(fYg * fYg + fZg * fZg)) * 180.0f) / 3.1415f;
			
			rollAvg += roll[avgIndex];
			pitchAvg += pitch[avgIndex];
			avgIndex++;
			
			if(avgIndex >= PROCESS_NUM)
			{
				rollAvg = rollAvg / avgIndex;
				pitchAvg = pitchAvg / avgIndex;
				
				for(int i = 0; i < PROCESS_NUM ; i++)
				{
					if(fabs(rollAvg) - fabs(roll[i]) > DETECTION_THRESHOLD)
					{
						ACCL_LOG("ACCL_LOG | *** ACTIVITY *** \r\n");
						uAcclActiveState = 1;	
						uwDeactiveTM = HAL_GetTick();
						break;
					}
				}
				
				for(int i = 0; i < PROCESS_NUM ; i++)
				{
					if(fabs(pitchAvg) - fabs(pitch[i]) > DETECTION_THRESHOLD)
					{
						ACCL_LOG("ACCL_LOG | *** ACTIVITY *** \r\n");
						uAcclActiveState = 1;	
						uwDeactiveTM = HAL_GetTick();
						break;
					}
				}				
				
				// Output Results to Serial
				rollAvgOld = rollAvg;
				pitchAvgOld = pitchAvg;
				ACCL_LOG("ACCL_LOG | roll: %f | pitch: %f \r\n", rollAvg, pitchAvg);
				avgIndex = 0;
				rollAvg = 0;
				pitchAvg = 0;
			}
			
			if((HAL_GetTick() - uwDeactiveTM > DEACTIVE_TIME * 1000) && (uAcclActiveState == 1)){
				ACCL_LOG("ACCL_LOG | *** INACTIVITY *** \r\n");
				uAcclActiveState = 0;	
			}
	}	
}


//=============================
double ADXL345_AcclGetRoll(void)
//=============================
{
	return rollAvgOld;
}

//=============================
double ADXL345_AcclGetPitch(void)
//=============================
{
	return pitchAvgOld;
}

//================================
uint8_t ADXL345_AcclGetActivity(void)
//================================
{
	return uAcclActiveState;
}
