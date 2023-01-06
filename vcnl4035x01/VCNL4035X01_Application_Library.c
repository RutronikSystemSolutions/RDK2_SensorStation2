/*
 * VCNL4035X01_Application_Library.c
 *
 * Created  : 9 November 2020
 * Modified : 6 May 2021
 * Author   : HWanyusof
 * Version	: 1.0
 */

#include "VCNL4035X01_Prototypes.h"
#include "VCNL4035X01.h"
#include "I2C_Functions.h"
#include "VCNL4035X01_Application_Library.h"

extern int I2C_Bus;

//****************************************************************************************************
//***************************************Application API**********************************************

/*Determine Sensitivity From ALS IT
 *VCNL4035X01_CAL_Sensitivity(Byte ALS_IT)
 *Byte ALS_IT - Input Parameter:
 *
 * VCNL4035X01_ALS_IT_50MS
 * VCNL4035X01_ALS_IT_100MS
 * VCNL4035X01_ALS_IT_200MS
 * VCNL4035X01_ALS_IT_400MS
 * VCNL4035X01_ALS_IT_800MS
 */
float VCNL4035X01_CAL_Sensitivity(Byte ALS_IT)
{
    float Sensitivity;

	if (ALS_IT == VCNL4035X01_ALS_IT_50MS) {Sensitivity = 0.064;}
	if (ALS_IT == VCNL4035X01_ALS_IT_100MS) {Sensitivity = 0.032;}
	if (ALS_IT == VCNL4035X01_ALS_IT_200MS) {Sensitivity = 0.016;}
	if (ALS_IT == VCNL4035X01_ALS_IT_400MS) {Sensitivity = 0.008;}
	if (ALS_IT == VCNL4035X01_ALS_IT_800MS) {Sensitivity = 0.004;}

	return(Sensitivity);
}

/*Calculate the ALS (Lux)
 *VCNL4035X01_CAL_Lux(float Sensitivity, float Count)
 *float Sensitivity - output from VCNL4035X01_CAL_Sensitivity(Byte ALS_IT)
 *float Count - output from VCNL4035X01_GET_ALS_Data()
 */
float VCNL4035X01_CAL_Lux(float Sensitivity, float Count)
{
    float Lux;
    Lux = Sensitivity*Count;
    return(Lux);
}

/*Get Delay for Measurement
 *VCNL4035X01_GET_Delay(Byte ALS_IT)
 *Byte ALS_IT - Input Parameter:
 *
 * VCNL4035X01_ALS_IT_50MS
 * VCNL4035X01_ALS_IT_100MS
 * VCNL4035X01_ALS_IT_200MS
 * VCNL4035X01_ALS_IT_400MS
 * VCNL4035X01_ALS_IT_800MS
 */
int VCNL4035X01_GET_Delay(Byte ALS_IT)
{
	int Delay;

	//Delay for ALS
	//Delay = Delay from IT + Circuit (~10ms)
	if (ALS_IT == VCNL4035X01_ALS_IT_50MS) {Delay = 60;}
	if (ALS_IT == VCNL4035X01_ALS_IT_100MS) {Delay = 110;}
	if (ALS_IT == VCNL4035X01_ALS_IT_200MS) {Delay = 210;}
	if (ALS_IT == VCNL4035X01_ALS_IT_400MS) {Delay = 410;}
	if (ALS_IT == VCNL4035X01_ALS_IT_800MS) {Delay = 810;}

	return Delay;
}

/*Get Proximity Mode
 *returns the PS mode status of the sensor as follows:
 *
 * 0 - PS Shutdown Mode
 * 1 - Auto/Self-Timed Mode
 * 2 - Active Force Mode/Continuous Forced Mode
 * 3 - Gesture Mode
 * 4 - Low Power Mode
 *
 */
int VCNL4035X01_GET_PS_Mode()
{
	int Mode;

	//Read the PS_SD bit: Mode = 0 - PS Shutdown
	if (VCNL4035X01_GET_PS_SD_Bit() == 0b1) Mode = 0;

	//Read the PS_AF bit and PS_SD: Mode = 1 - Auto/Self-Timed Mode
	if ((VCNL4035X01_GET_PS_AF_Bit() == 0b0) && (VCNL4035X01_GET_PS_SD_Bit() == 0b0)) Mode = 1;

	//Read the PS_AF bit: Mode = 2 - AF Mode
	if (VCNL4035X01_GET_PS_AF_Bit() == 0b1) Mode = 2;

	//Read the GESTURE_MODE bit: Mode = 3 - Gesture Mode
	if (VCNL4035X01_GET_GESTURE_MODE_Bit() == 0b1) Mode = 3;

	//Read the LED_I_LOW bit: Mode = 4 - Low Power Mode
	if (VCNL4035X01_GET_LED_I_LOW_Bit() == 0b1) Mode = 4;

	return Mode;
}

/*Get ALS Mode
 *returns the ALS mode status of the sensor as follows:
 *
 * 0 - ALS Shutdown Mode
 * 1 - ALS Mode
 * 2 - White Channel Mode
 *
 */
int VCNL4035X01_GET_ALS_Mode()
{
	int Mode;

	//Read the ALS_SD bit: Mode = 0 - ALS Shutdown
	if (VCNL4035X01_GET_ALS_SD_Bit() == 0b1) Mode = 0;

	//Read the WHITE_SD and ALS_SD bit: Mode = 1 - ALS Mode
	if ((VCNL4035X01_GET_WHITE_SD_Bit() == 0b1) && (VCNL4035X01_GET_ALS_SD_Bit() == 0b0)) Mode = 1;

	//Read the WHITE_SD bit: Mode = 2 - White Mode
	if (VCNL4035X01_GET_WHITE_SD_Bit() == 0b0) Mode = 2;

	return Mode;
}

//Reset the Sensor to the default value
void Reset_Sensor()
{
	struct TransferData VCNL4035X01_Data;
	VCNL4035X01_Data.Slave_Address = VCNL4035X01_SlaveAddress;
	VCNL4035X01_Data.RegisterAddress = VCNL4035X01_ALS_CONF_1;
	VCNL4035X01_Data.Select_I2C_Bus = I2C_Bus;
	VCNL4035X01_Data.WData[0] = 0x01;
	VCNL4035X01_Data.WData[1] = 0x01;
	WriteI2C_Bus(&VCNL4035X01_Data);

	VCNL4035X01_Data.Slave_Address = VCNL4035X01_SlaveAddress;
	VCNL4035X01_Data.RegisterAddress = VCNL4035X01_ALS_THDL;
	VCNL4035X01_Data.Select_I2C_Bus = I2C_Bus;
	VCNL4035X01_Data.WData[0] = 0x00;
	VCNL4035X01_Data.WData[1] = 0x00;
	WriteI2C_Bus(&VCNL4035X01_Data);

	VCNL4035X01_Data.Slave_Address = VCNL4035X01_SlaveAddress;
	VCNL4035X01_Data.RegisterAddress = VCNL4035X01_ALS_THDH;
	VCNL4035X01_Data.Select_I2C_Bus = I2C_Bus;
	VCNL4035X01_Data.WData[0] = 0x00;
	VCNL4035X01_Data.WData[1] = 0x00;
	WriteI2C_Bus(&VCNL4035X01_Data);

	VCNL4035X01_Data.Slave_Address = VCNL4035X01_SlaveAddress;
	VCNL4035X01_Data.RegisterAddress = VCNL4035X01_PS_CONF_1;
	VCNL4035X01_Data.Select_I2C_Bus = I2C_Bus;
	VCNL4035X01_Data.WData[0] = 0x01;
	VCNL4035X01_Data.WData[1] = 0x00;
	WriteI2C_Bus(&VCNL4035X01_Data);

	VCNL4035X01_Data.Slave_Address = VCNL4035X01_SlaveAddress;
	VCNL4035X01_Data.RegisterAddress = VCNL4035X01_PS_CONF_3;
	VCNL4035X01_Data.Select_I2C_Bus = I2C_Bus;
	VCNL4035X01_Data.WData[0] = 0x00;
	VCNL4035X01_Data.WData[1] = 0x00;
	WriteI2C_Bus(&VCNL4035X01_Data);

    VCNL4035X01_Data.Slave_Address = VCNL4035X01_SlaveAddress;
	VCNL4035X01_Data.RegisterAddress = VCNL4035X01_PS_CANC;
	VCNL4035X01_Data.Select_I2C_Bus = I2C_Bus;
	VCNL4035X01_Data.WData[0] = 0x00;
	VCNL4035X01_Data.WData[1] = 0x00;
	WriteI2C_Bus(&VCNL4035X01_Data);

	VCNL4035X01_Data.Slave_Address = VCNL4035X01_SlaveAddress;
	VCNL4035X01_Data.RegisterAddress = VCNL4035X01_PS_THDL;
	VCNL4035X01_Data.Select_I2C_Bus = I2C_Bus;
	VCNL4035X01_Data.WData[0] = 0x00;
	VCNL4035X01_Data.WData[1] = 0x00;
	WriteI2C_Bus(&VCNL4035X01_Data);

	VCNL4035X01_Data.Slave_Address = VCNL4035X01_SlaveAddress;
	VCNL4035X01_Data.RegisterAddress = VCNL4035X01_PS_THDH;
	VCNL4035X01_Data.Select_I2C_Bus = I2C_Bus;
	VCNL4035X01_Data.WData[0] = 0x00;
	VCNL4035X01_Data.WData[1] = 0x00;
	WriteI2C_Bus(&VCNL4035X01_Data);

	VCNL4035X01_Data.Slave_Address = VCNL4035X01_SlaveAddress;
	VCNL4035X01_Data.RegisterAddress = VCNL4035X01_INT_FLAG;
	VCNL4035X01_Data.Select_I2C_Bus = I2C_Bus;
	VCNL4035X01_Data.WData[0] = 0x00;
	VCNL4035X01_Data.WData[1] = 0x00;
	WriteI2C_Bus(&VCNL4035X01_Data);
}

/*Print all of the Register Value for both proximity and ambient light sensing functions
 *Print_All(Byte ALS_IT)
 *Byte ALS_IT - Input Parameter:
 *
 * VCNL4035X01_ALS_IT_50MS
 * VCNL4035X01_ALS_IT_100MS
 * VCNL4035X01_ALS_IT_200MS
 * VCNL4035X01_ALS_IT_400MS
 * VCNL4035X01_ALS_IT_800MS
 */
void Print_All(Byte ALS_IT)
{
	Word value;
	Word Data1,Data2,Data3;
    bool Gesture_Data_Ready;
	int Delay;
	float Sensitivity;
	float Lux;

	#ifdef STM32F

		#define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Print ALS Mode
    	switch(VCNL4035X01_GET_ALS_Mode())
    	{
    		case 0 :	sprintf(TransmitBuffer2,">>>>>>>Mode : ALS Off/Shutdown \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 1 :	sprintf(TransmitBuffer2,">>>>>>>Mode : ALS Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 2 :	sprintf(TransmitBuffer2,">>>>>>>Mode : White Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    	}

    	sprintf(TransmitBuffer2,">>>>>>>>>>>>>>>>>>>>>>>ALS<<<<<<<<<<<<<<<<<<<<<<<<< \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Print the Reg 0x00 (ALS_CONF1 and ALS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_CONF_1);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x00 (ALS_CONF) Val : 0x%x \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x01 (ALS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDH);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x01 (ALS_THDH) Val : %d Counts\r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x02 (ALS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDL);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x02 (ALS_THDL) Val : %d Counts\r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Get Delay for ALS Measurement
    	Delay = VCNL4035X01_GET_Delay(ALS_IT);

    	//Delay of IT ms + other Circuit Delay (~10ms)
    	HAL_Delay(Delay);

    	//Print ALS Data
    	if(VCNL4035X01_GET_ALS_Mode() == 1)
    	{
    		//Find Sensitivity
    		Sensitivity = VCNL4035X01_CAL_Sensitivity(ALS_IT);

    		//Print the Sensitivity
    		ftoa(Sensitivity,TransmitBuffer, 6);
    		sprintf(TransmitBuffer2,">>>>>>>Sensitivity : %s lx/Count<<<<<<<<\r\n",TransmitBuffer);
    		CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    		HAL_Delay(50);

    		//Print the ALS Data in DEC
    		value = VCNL4035X01_GET_ALS_Data();
    		sprintf(TransmitBuffer,">>>>>>>ALS Data : %d Counts<<<<<<<<\r\n",value);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);

    		//Calculate Lux
    		Lux = VCNL4035X01_CAL_Lux(Sensitivity, value);

    		//Print the ALS Lux in DEC
    		ftoa(Lux,TransmitBuffer, 5);
    		sprintf(TransmitBuffer2,">>>>>>>ALS Lux : %s lx<<<<<<<<\r\n",TransmitBuffer);
    		CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    		HAL_Delay(50);
    	}

    	//Print White Data
    	if(VCNL4035X01_GET_ALS_Mode() == 2)
    	{
    		//Print the White Data in DEC
    		value = VCNL4035X01_GET_White_Data();
    		sprintf(TransmitBuffer,">>>>>>>White Data : %d Counts<<<<<<<<\r\n",value);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);
    	}

    	//Print the Interrupt Flag
    	//Only print here when PS is off/shutdown and ALS/White channel is on
    	if((VCNL4035X01_GET_PS_Mode() == 0) && (VCNL4035X01_GET_ALS_Mode() != 0))
    	{
    		sprintf(TransmitBuffer2,"*************************************************** \r\n");
    		CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    		HAL_Delay(50);

    		value = VCNL4035X01_GET_ALS_Interrupt();
    		sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);
    	}

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Print PS Mode
    	switch(VCNL4035X01_GET_PS_Mode())
    	{
    		case 0 :	sprintf(TransmitBuffer2,">>>>>>>Mode : PS Off/Shutdown \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 1 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Auto/Self-Timed Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 2 :	sprintf(TransmitBuffer2,">>>>>>>Mode : AF/Continuous Forced Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 3 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Gesture Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 4 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Low Power Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    	}

    	sprintf(TransmitBuffer2,">>>>>>>>>>>>>>>>>>>>>>>>PS<<<<<<<<<<<<<<<<<<<<<<<<< \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Print the Reg 0x03 (PS_CONF1 and PS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_1);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x03 (PS_CONF1) Val : 0x%x \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x04 (PS_CONF3 and PS_MS)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_3);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x04 (PS_CONF3) Val : 0x%x \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x05 (PS_CANC)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CANC);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x05 (PS_CANC) Val : %d Counts \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x06 (PS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDL);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x06 (PS_THDL) Val : %d Counts \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x07 (PS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDH);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x07 (PS_THDH) Val : %d Counts \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print Proximity Data (Only For Auto Mode, AF and Low Power Mode)
    	if((VCNL4035X01_GET_PS_Mode() > 0) && (VCNL4035X01_GET_PS_Mode() < 5) && ((VCNL4035X01_GET_PS_Mode() != 3)))
    	{
    		//Set Trigger for AF Mode + Associated delay due to IT
    		if(VCNL4035X01_GET_PS_Mode() == 2)
    		{
    			//Set trigger to start a measurement
    			VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    			//Delay of PS Measurement + other Circuit Delay
    			HAL_Delay(50);
    		}

    		//Delay for Auto and Low Power Mode + Associated delay due to IT
    		if((VCNL4035X01_GET_PS_Mode() == 1) || (VCNL4035X01_GET_PS_Mode() == 4))
    		{
    			//Delay of PS Measurement + other Circuit Delay
    			HAL_Delay(50);
    		}

    		//Print the Proximity Data (IRED 1)
    		value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
    		sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",value);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);

    		//Print the Proximity Data (IRED 2)
    		value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
    		sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",value);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);

    		//Print the Proximity Data (IRED 3)
    		value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
    		sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",value);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);
    	}

    	//Print Gesture Data (Only Gesture Mode)
    	if((VCNL4035X01_GET_PS_Mode() == 3) && ((VCNL4035X01_GET_PS_Mode() != 0)))
    	{
    		//Set trigger to start a measurement
    		VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    		//Delay of PS Measurement + other Circuit Delay
    		HAL_Delay(50);

    		//Print the Gesture Data Ready Interrupt Flag
    		Gesture_Data_Ready = VCNL4035X01_GET_Gesture_Data_Ready_Flag();
    		//Need to convert to int since sprintf has no bool specifier
    		sprintf(TransmitBuffer,">>>>>>>Gesture Data Ready INT Flag : 0x%x<<<<<<<< \r\n",(int)Gesture_Data_Ready);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);

    		//Read Gesture Data
    		VCNL4035X01_GET_Gesture_Mode_Data(&Data1, &Data2, &Data3);

    		//Print the Proximity Data (IRED 1)
    		sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",Data1);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);

    		//Print the Proximity Data (IRED 2)
    		sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",Data2);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);

    		//Print the Proximity Data (IRED 3)
    		sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",Data3);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);
    	}

    	//Print the Interrupt Flag
    	if(((VCNL4035X01_GET_ALS_Mode() != 0) && (VCNL4035X01_GET_PS_Mode() != 0)) || ((VCNL4035X01_GET_ALS_Mode() == 0) && (VCNL4035X01_GET_PS_Mode() == 0)) || ((VCNL4035X01_GET_ALS_Mode() == 0) && (VCNL4035X01_GET_PS_Mode() != 0)))
    	{
    		//Do not print Interrupt for Gesture Mode
    		if (VCNL4035X01_GET_PS_Mode() != 3)
    		{
    			sprintf(TransmitBuffer2,"*************************************************** \r\n");
    			CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    			HAL_Delay(50);

    			//Print the Interrupt Flag
    			value = VCNL4035X01_GET_PS_Interrupt();
    			sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
    			CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    			HAL_Delay(50);
    		}
    	}

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	HAL_Delay(2000);

	#endif

    #ifdef Cypress

        #define TRANSMIT_BUFFER_SIZE  128
		char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
		char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

		if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        //Print ALS Mode
        if(CDC_CDCIsReady() != 0u)
    	{
        	switch(VCNL4035X01_GET_ALS_Mode())
        	{
        		case 0 :	sprintf(TransmitBuffer2,">>>>>>>Mode : ALS Off/Shutdown \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 1 :	sprintf(TransmitBuffer2,">>>>>>>Mode : ALS Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 2 :	sprintf(TransmitBuffer2,">>>>>>>Mode : White Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        	}
        }
        CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,">>>>>>>>>>>>>>>>>>>>>>>ALS<<<<<<<<<<<<<<<<<<<<<<<<< \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

    	//Print the Reg 0x00 (ALS_CONF1 and ALS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_CONF_1);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x00 (ALS_CONF) Val : 0x%x \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x01 (ALS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDH);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x01 (ALS_THDH) Val : %d Counts\r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x02 (ALS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDL);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x02 (ALS_THDL) Val : %d Counts\r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Get Delay for ALS Measurement
    	Delay = VCNL4035X01_GET_Delay(ALS_IT);

    	//Delay of IT ms + other Circuit Delay (~10ms)
    	CyDelay(Delay);

    	//Print ALS Data
    	if(VCNL4035X01_GET_ALS_Mode() == 1)
    	{
    		//Find Sensitivity
    		Sensitivity = VCNL4035X01_CAL_Sensitivity(ALS_IT);

    		//Print the Sensitivity
    		ftoa(Sensitivity,TransmitBuffer, 6);
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer2,">>>>>>>Sensitivity : %s lx/Count<<<<<<<<\r\n",TransmitBuffer);
        		CDC_PutString(TransmitBuffer2);
            }
    		CyDelay(50);

    		//Print the ALS Data in DEC
    		value = VCNL4035X01_GET_ALS_Data();
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>ALS Data : %d Counts<<<<<<<<\r\n",value);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);

    		//Calculate Lux
    		Lux = VCNL4035X01_CAL_Lux(Sensitivity, value);

    		//Print the ALS Lux in DEC
    		ftoa(Lux,TransmitBuffer, 5);
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer2,">>>>>>>ALS Lux : %s lx<<<<<<<<\r\n",TransmitBuffer);
        		CDC_PutString(TransmitBuffer2);
            }
    		CyDelay(50);
    	}

    	//Print White Data
    	if(VCNL4035X01_GET_ALS_Mode() == 2)
    	{
    		//Print the White Data in DEC
    		value = VCNL4035X01_GET_White_Data();
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>White Data : %d Counts<<<<<<<<\r\n",value);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);
    	}

    	//Print the Interrupt Flag
    	//Only print here when PS is off/shutdown and ALS/White channel is on
    	if((VCNL4035X01_GET_PS_Mode() == 0) && (VCNL4035X01_GET_ALS_Mode() != 0))
    	{
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer2,"*************************************************** \r\n");
        		CDC_PutString(TransmitBuffer2);
            }
    		CyDelay(50);

    		value = VCNL4035X01_GET_ALS_Interrupt();
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);
    	}

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	//Print PS Mode
        	switch(VCNL4035X01_GET_PS_Mode())
        	{
        		case 0 :	sprintf(TransmitBuffer2,">>>>>>>Mode : PS Off/Shutdown \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 1 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Auto/Self-Timed Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 2 :	sprintf(TransmitBuffer2,">>>>>>>Mode : AF/Continuous Forced Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 3 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Gesture Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 4 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Low Power Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        	}
        }
        CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,">>>>>>>>>>>>>>>>>>>>>>>>PS<<<<<<<<<<<<<<<<<<<<<<<<< \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

    	//Print the Reg 0x03 (PS_CONF1 and PS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_1);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x03 (PS_CONF1) Val : 0x%x \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x04 (PS_CONF3 and PS_MS)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_3);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x04 (PS_CONF3) Val : 0x%x \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x05 (PS_CANC)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CANC);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x05 (PS_CANC) Val : %d Counts \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x06 (PS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDL);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x06 (PS_THDL) Val : %d Counts \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x07 (PS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDH);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x07 (PS_THDH) Val : %d Counts \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print Proximity Data (Only For Auto Mode, AF and Low Power Mode)
    	if((VCNL4035X01_GET_PS_Mode() > 0) && (VCNL4035X01_GET_PS_Mode() < 5) && ((VCNL4035X01_GET_PS_Mode() != 3)))
    	{
    		//Set Trigger for AF Mode + Associated delay due to IT
    		if(VCNL4035X01_GET_PS_Mode() == 2)
    		{
    			//Set trigger to start a measurement
    			VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    			//Delay of PS Measurement + other Circuit Delay
    			CyDelay(50);
    		}

    		//Delay for Auto and Low Power Mode + Associated delay due to IT
    		if((VCNL4035X01_GET_PS_Mode() == 1) || (VCNL4035X01_GET_PS_Mode() == 4))
    		{
    			//Delay of PS Measurement + other Circuit Delay
    			CyDelay(50);
    		}

    		//Print the Proximity Data (IRED 1)
    		value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",value);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);

    		//Print the Proximity Data (IRED 2)
    		value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",value);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);

    		//Print the Proximity Data (IRED 3)
    		value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",value);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);
    	}

    	//Print Gesture Data (Only Gesture Mode)
    	if((VCNL4035X01_GET_PS_Mode() == 3) && ((VCNL4035X01_GET_PS_Mode() != 0)))
    	{
    		//Set trigger to start a measurement
    		VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    		//Delay of PS Measurement + other Circuit Delay
    		CyDelay(50);

    		//Print the Gesture Data Ready Interrupt Flag
    		Gesture_Data_Ready = VCNL4035X01_GET_Gesture_Data_Ready_Flag();
            if(CDC_CDCIsReady() != 0u)
    	    {
        		//Need to convert to int since sprintf has no bool specifier
        		sprintf(TransmitBuffer,">>>>>>>Gesture Data Ready INT Flag : 0x%x<<<<<<<< \r\n",(int)Gesture_Data_Ready);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);

    		//Read Gesture Data
    		VCNL4035X01_GET_Gesture_Mode_Data(&Data1, &Data2, &Data3);

    		//Print the Proximity Data (IRED 1)
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",Data1);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);

    		//Print the Proximity Data (IRED 2)
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",Data2);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);

    		//Print the Proximity Data (IRED 3)
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",Data3);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);
    	}

    	//Print the Interrupt Flag
    	if(((VCNL4035X01_GET_ALS_Mode() != 0) && (VCNL4035X01_GET_PS_Mode() != 0)) || ((VCNL4035X01_GET_ALS_Mode() == 0) && (VCNL4035X01_GET_PS_Mode() == 0)) || ((VCNL4035X01_GET_ALS_Mode() == 0) && (VCNL4035X01_GET_PS_Mode() != 0)))
    	{
    		//Do not print Interrupt for Gesture Mode
    		if (VCNL4035X01_GET_PS_Mode() != 3)
    		{
                if(CDC_CDCIsReady() != 0u)
    	        {
        			sprintf(TransmitBuffer2,"*************************************************** \r\n");
        			CDC_PutString(TransmitBuffer2);
                }
    			CyDelay(50);

    			//Print the Interrupt Flag
    			value = VCNL4035X01_GET_PS_Interrupt();
                if(CDC_CDCIsReady() != 0u)
    	        {
        			sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
        			CDC_PutString(TransmitBuffer);
                }
    			CyDelay(50);
    		}
    	}

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	CyDelay(2000);

    #endif

    #ifdef Arduino

		Serial.println("***************************************************");
    	delay(50);

        //Print ALS Mode
    	switch(VCNL4035X01_GET_ALS_Mode())
    	{
    		case 0 :	Serial.println(">>>>>>>Mode : ALS Off/Shutdown");
    					break;
    		case 1 :	Serial.println(">>>>>>>Mode : ALS Mode");
    					break;
    		case 2 :	Serial.println(">>>>>>>Mode : White Mode");
    					break;
    	}
        delay(50);

    	Serial.println(">>>>>>>>>>>>>>>>>>>>>>>ALS<<<<<<<<<<<<<<<<<<<<<<<<<");
    	delay(50);

    	//Print the Reg 0x00 (ALS_CONF1 and ALS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_CONF_1);
    	Serial.print(">>>>>>>Reg 0x00 (ALS_CONF) Val : 0x");
    	Serial.println(value,HEX);
    	delay(50);

    	//Print the Reg 0x01 (ALS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDH);
    	Serial.print(">>>>>>>Reg 0x01 (ALS_THDH) Val : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts ");
    	delay(50);

    	//Print the Reg 0x02 (ALS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDL);
    	Serial.print(">>>>>>>Reg 0x02 (ALS_THDL) Val : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts ");
    	delay(50);

    	//Get Delay for ALS Measurement
    	Delay = VCNL4035X01_GET_Delay(ALS_IT);

    	//Delay of IT ms + other Circuit Delay (~10ms)
    	delay(Delay);

    	//Print ALS Data
    	if(VCNL4035X01_GET_ALS_Mode() == 1)
    	{
    		//Find Sensitivity
    		Sensitivity = VCNL4035X01_CAL_Sensitivity(ALS_IT);

    		//Print the Sensitivity
    		Serial.print(">>>>>>>Sensitivity : ");
    		Serial.print(Sensitivity,5);
    		Serial.println(" lx/count<<<<<<<<");
    		delay(50);

    		//Print the ALS Data in DEC
    		value = VCNL4035X01_GET_ALS_Data();
            Serial.print(">>>>>>>ALS Data : ");
    		Serial.print(value,DEC);
    		Serial.println(" Counts<<<<<<<<");
    		delay(50);

    		//Calculate Lux
    		Lux = VCNL4035X01_CAL_Lux(Sensitivity, value);

    		//Print the ALS Lux
    		Serial.print(">>>>>>>ALS Lux : ");
    		Serial.print(Lux,5);
    		Serial.println(" lx<<<<<<<<");
    		delay(50);
    	}

    	//Print White Data
    	if(VCNL4035X01_GET_ALS_Mode() == 2)
    	{
    		//Print the White Data in DEC
    		value = VCNL4035X01_GET_White_Data();
    		Serial.print(">>>>>>>White Data : ");
    		Serial.print(value,DEC);
    		Serial.println(" Counts<<<<<<<<");
    		delay(50);
    	}

    	//Print the Interrupt Flag
    	//Only print here when PS is off/shutdown and ALS/White channel is on
    	if((VCNL4035X01_GET_PS_Mode() == 0) && (VCNL4035X01_GET_ALS_Mode() != 0))
    	{
    		Serial.println("***************************************************");
    		delay(50);

    		value = VCNL4035X01_GET_ALS_Interrupt();

    		Serial.print(">>>>>>>Interrupt Flag : 0b");
    		Serial.print(value,BIN);
    		Serial.println("<<<<<<<<");
    		delay(50);
    	}

    	Serial.println("***************************************************");
    	delay(50);


    	//Print PS Mode
    	switch(VCNL4035X01_GET_PS_Mode())
    	{
    		case 0 :	Serial.println(">>>>>>>Mode : PS Off/Shutdown");
    					break;
    		case 1 :	Serial.println(">>>>>>>Mode : Auto/Self-Timed Mode");
    					break;
    		case 2 :	Serial.println(">>>>>>>Mode : AF/Continuous Forced Mode");
    					break;
    		case 3 :	Serial.println(">>>>>>>Mode : Gesture Mode");
    					break;
    		case 4 :	Serial.println(">>>>>>>Mode : Low Power Mode");
    					break;
    	}
        delay(50);

    	Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>PS<<<<<<<<<<<<<<<<<<<<<<<<<");
    	delay(50);

    	//Print the Reg 0x03 (PS_CONF1 and PS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_1);
    	Serial.print(">>>>>>>Reg 0x03 (PS_CONF1) Val : 0x");
    	Serial.println(value,HEX);
    	delay(50);

    	//Print the Reg 0x04 (PS_CONF3 and PS_MS)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_3);
    	Serial.print(">>>>>>>Reg 0x04 (PS_CONF3) Val : 0x");
    	Serial.println(value,HEX);
    	delay(50);

    	//Print the Reg 0x05 (PS_CANC)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CANC);
    	Serial.print(">>>>>>>Reg 0x05 (PS_CANC) Val : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts ");
    	delay(50);

    	//Print the Reg 0x06 (PS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDL);
    	Serial.print(">>>>>>>Reg 0x06 (PS_THDL) Val : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts ");
    	delay(50);

    	//Print the Reg 0x07 (PS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDH);
    	Serial.print(">>>>>>>Reg 0x07 (PS_THDH) Val : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts ");
    	delay(50);

    	//Print Proximity Data (Only For Auto Mode, AF and Low Power Mode)
    	if((VCNL4035X01_GET_PS_Mode() > 0) && (VCNL4035X01_GET_PS_Mode() < 5) && ((VCNL4035X01_GET_PS_Mode() != 3)))
    	{
    		//Set Trigger for AF Mode + Associated delay due to IT
    		if(VCNL4035X01_GET_PS_Mode() == 2)
    		{
    			//Set trigger to start a measurement
    			VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    			//Delay of PS Measurement + other Circuit Delay
    			delay(50);
    		}

    		//Delay for Auto and Low Power Mode + Associated delay due to IT
    		if((VCNL4035X01_GET_PS_Mode() == 1) || (VCNL4035X01_GET_PS_Mode() == 4))
    		{
    			//Delay of PS Measurement + other Circuit Delay
    			delay(50);
    		}

    		//Print the Proximity Data (IRED 1)
    		Data1 = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
    		Serial.print(">>>>>>>Proximity Data 1 : ");
    		Serial.print(Data1,DEC);
    		Serial.println(" Counts<<<<<<<<");
    		delay(50);

    		//Print the Proximity Data (IRED 2)
    		Data2 = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
    		Serial.print(">>>>>>>Proximity Data 2 : ");
    		Serial.print(Data2,DEC);
    		Serial.println(" Counts<<<<<<<<");
    		delay(50);

    		//Print the Proximity Data (IRED 3)
    		Data3 = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
    		Serial.print(">>>>>>>Proximity Data 3 : ");
    		Serial.print(Data3,DEC);
    		Serial.println(" Counts<<<<<<<<");
    		delay(50);
    	}

    	//Print Gesture Data (Only Gesture Mode)
    	if((VCNL4035X01_GET_PS_Mode() == 3) && ((VCNL4035X01_GET_PS_Mode() != 0)))
    	{
    		//Set trigger to start a measurement
    		VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    		//Delay of PS Measurement + other Circuit Delay
    		delay(50);

    		//Print the Gesture Data Ready Interrupt Flag
    		Gesture_Data_Ready = VCNL4035X01_GET_Gesture_Data_Ready_Flag();
    		Serial.print(">>>>>>>Gesture Data Ready INT Flag : 0b");
    		Serial.print(Gesture_Data_Ready,BIN);
    		Serial.println("<<<<<<<<");
    		delay(50);

    		//Read Gesture Data
    		VCNL4035X01_GET_Gesture_Mode_Data(&Data1, &Data2, &Data3);

    		//Print the Proximity Data (IRED 1)
    		Serial.print(">>>>>>>Proximity Data 1 : ");
    		Serial.print(Data1,DEC);
    		Serial.println(" Counts<<<<<<<<");
    		delay(50);

    		//Print the Proximity Data (IRED 2)
    		Serial.print(">>>>>>>Proximity Data 2 : ");
    		Serial.print(Data2,DEC);
    		Serial.println(" Counts<<<<<<<<");
    		delay(50);

    		//Print the Proximity Data (IRED 3)
    		Serial.print(">>>>>>>Proximity Data 3 : ");
    		Serial.print(Data3,DEC);
    		Serial.println(" Counts<<<<<<<<");
    		delay(50);
    	}

    	//Print the Interrupt Flag
    	if(((VCNL4035X01_GET_ALS_Mode() != 0) && (VCNL4035X01_GET_PS_Mode() != 0)) || ((VCNL4035X01_GET_ALS_Mode() == 0) && (VCNL4035X01_GET_PS_Mode() == 0)) || ((VCNL4035X01_GET_ALS_Mode() == 0) && (VCNL4035X01_GET_PS_Mode() != 0)))
    	{
    		//Do not print Interrupt for Gesture Mode
    		if (VCNL4035X01_GET_PS_Mode() != 3)
    		{
    			Serial.println("***************************************************");
    			delay(50);

    	        //Print the Interrupt Flag
    			value = VCNL4035X01_GET_PS_Interrupt();
    			Serial.print(">>>>>>>Interrupt Flag : 0b");
    			Serial.print(value,BIN);
    			Serial.println("<<<<<<<<");
    			delay(50);
    		}
    	}

    	Serial.println("***************************************************");
    	delay(50);

    	Serial.println("");
    	delay(50);

    	Serial.println("");

    	delay(2000);

    #endif
}

//Print the output of the Auto Mode/Self-Timed Mode
void Print_Auto_Mode()
{
	Word value;

    #ifdef STM32F

        #define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"****************Auto/Self-Timed Mode*************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	//Delay of PS Measurement + other Circuit Delay
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 1)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 3)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Interrupt Flag
    	value = VCNL4035X01_GET_PS_Interrupt();
    	sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	HAL_Delay(2000);

    #endif

    #ifdef Cypress

    	#define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"****************Auto/Self-Timed Mode*************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	//Delay of PS Measurement + other Circuit Delay
    	CyDelay(50);

    	//Print the Proximity Data (IRED 1)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Proximity Data (IRED 2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Proximity Data (IRED 3)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Interrupt Flag
    	value = VCNL4035X01_GET_PS_Interrupt();
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	CyDelay(2000);

    #endif

    #ifdef Arduino

        Serial.println("***************************************************");
    	delay(50);

    	Serial.println("****************Auto/Self-Timed Mode***************");

    	//Delay of PS Measurement + other Circuit Delay
    	delay(50);

    	//Print the Proximity Data (IRED 1)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
    	Serial.print(">>>>>>>Proximity Data 1 : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Proximity Data (IRED 2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
    	Serial.print(">>>>>>>Proximity Data 2 : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Proximity Data (IRED 3)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
    	Serial.print(">>>>>>>Proximity Data 3 : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Interrupt Flag
    	value = VCNL4035X01_GET_PS_Interrupt();
    	Serial.print(">>>>>>>Interrupt Flag : 0b");
    	Serial.print(value,BIN);
    	Serial.println("<<<<<<<<");
    	delay(50);

    	Serial.println("***************************************************");
    	delay(50);

    	Serial.println("");
    	delay(50);

    	Serial.println("");

    	delay(2000);
    #endif
}

//Print the output of the Active Force Mode/Continuous Forced Mode
void Print_AF_Mode()
{
	Word value;

    #ifdef STM32F

        #define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*************AF/Continuous Forced Mode************* \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	//Set trigger to start a measurement
    	VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    	//Delay of PS Measurement + other Circuit Delay
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 1)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 3)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Interrupt Flag
    	value = VCNL4035X01_GET_PS_Interrupt();
    	sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	HAL_Delay(2000);

    #endif

    #ifdef Cypress

    	#define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************AF/Continuous Forced Mode************* \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	//Set trigger to start a measurement
    	VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    	//Delay of PS Measurement + other Circuit Delay
    	CyDelay(50);

    	//Print the Proximity Data (IRED 1)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Proximity Data (IRED 2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Proximity Data (IRED 3)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Interrupt Flag
    	value = VCNL4035X01_GET_PS_Interrupt();
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	CyDelay(2000);

    #endif

    #ifdef Arduino

        Serial.println("***************************************************");
    	delay(50);

    	Serial.println("*************AF/Continuous Forced Mode*************");

    	//Set trigger to start a measurement
    	VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    	//Delay of PS Measurement + other Circuit Delay
    	delay(50);

    	//Print the Proximity Data (IRED 1)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
    	Serial.print(">>>>>>>Proximity Data 1 : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Proximity Data (IRED 2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
    	Serial.print(">>>>>>>Proximity Data 2 : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Proximity Data (IRED 3)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
    	Serial.print(">>>>>>>Proximity Data 3 : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Interrupt Flag
    	value = VCNL4035X01_GET_PS_Interrupt();
    	Serial.print(">>>>>>>Interrupt Flag : 0b");
    	Serial.print(value,BIN);
    	Serial.println("<<<<<<<<");
    	delay(50);

    	Serial.println("***************************************************");
    	delay(50);

    	Serial.println("");
    	delay(50);

    	Serial.println("");

    	delay(2000);

    #endif
}

//Print the output of the Gesture Mode
void Print_Gesture_Mode()
{
	Word Data1,Data2,Data3;
	bool Gesture_Data_Ready;

    #ifdef STM32F

        #define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"********************Gesture Mode******************* \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	//Set trigger to start a measurement
    	VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    	//Delay of PS Measurement + other Circuit Delay
    	HAL_Delay(50);

    	//Print the Gesture Data Ready Interrupt Flag
    	Gesture_Data_Ready = VCNL4035X01_GET_Gesture_Data_Ready_Flag();
    	//Need to convert to int since sprintf has no bool specifier
    	sprintf(TransmitBuffer,">>>>>>>Gesture Data Ready INT Flag : 0x%x<<<<<<<< \r\n",(int)Gesture_Data_Ready);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Read Gesture Data
    	VCNL4035X01_GET_Gesture_Mode_Data(&Data1, &Data2, &Data3);

    	//Print the Proximity Data (IRED 1)
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",Data1);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 2)
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",Data2);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 3)
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",Data3);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	HAL_Delay(2000);

    #endif

    #ifdef Cypress

    	#define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"********************Gesture Mode******************* \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	//Set trigger to start a measurement
    	VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    	//Delay of PS Measurement + other Circuit Delay
    	CyDelay(50);

    	//Print the Gesture Data Ready Interrupt Flag
    	Gesture_Data_Ready = VCNL4035X01_GET_Gesture_Data_Ready_Flag();
        if(CDC_CDCIsReady() != 0u)
    	{
        	//Need to convert to int since sprintf has no bool specifier
        	sprintf(TransmitBuffer,">>>>>>>Gesture Data Ready INT Flag : 0x%x<<<<<<<< \r\n",(int)Gesture_Data_Ready);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Read Gesture Data
    	VCNL4035X01_GET_Gesture_Mode_Data(&Data1, &Data2, &Data3);

    	//Print the Proximity Data (IRED 1)
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",Data1);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Proximity Data (IRED 2)
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",Data2);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Proximity Data (IRED 3)
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",Data3);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	CyDelay(2000);

    #endif

    #ifdef Arduino

        Serial.println("***************************************************");
    	delay(50);

    	Serial.println("********************Gesture Mode*******************");

    	//Set trigger to start a measurement
    	VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);

    	//Delay of PS Measurement + other Circuit Delay
    	delay(50);

    	//Print the Gesture Data Ready Interrupt Flag
    	Gesture_Data_Ready = VCNL4035X01_GET_Gesture_Data_Ready_Flag();
    	Serial.print(">>>>>>>Gesture Data Ready INT Flag : 0b");
    	Serial.print(Gesture_Data_Ready,BIN);
    	Serial.println("<<<<<<<<");
    	delay(50);

    	//Read Gesture Data
    	VCNL4035X01_GET_Gesture_Mode_Data(&Data1, &Data2, &Data3);

    	//Print the Proximity Data (IRED 1)
    	Serial.print(">>>>>>>Proximity Data 1 : ");
    	Serial.print(Data1,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Proximity Data (IRED 2)
    	Serial.print(">>>>>>>Proximity Data 2 : ");
    	Serial.print(Data2,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Proximity Data (IRED 3)
    	Serial.print(">>>>>>>Proximity Data 3 : ");
    	Serial.print(Data3,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	Serial.println("***************************************************");
    	delay(50);

    	Serial.println("");
    	delay(50);

    	Serial.println("");

    	delay(2000);

    #endif
}

//Print the output of the Low Power Mode
void Print_Low_Power_Mode()
{
	Word value;

    #ifdef STM32F

        #define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*******************Low Power Mode****************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	//Delay of PS Measurement + other Circuit Delay
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 1)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Proximity Data (IRED 3)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
    	sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Interrupt Flag
    	value = VCNL4035X01_GET_PS_Interrupt();
    	sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	HAL_Delay(2000);

    #endif

    #ifdef Cypress

    	#define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*******************Low Power Mode****************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	//Delay of PS Measurement + other Circuit Delay
    	CyDelay(50);

    	//Print the Proximity Data (IRED 1)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 1 : %d Counts<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Proximity Data (IRED 2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 2 : %d Counts<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Proximity Data (IRED 3)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Proximity Data 3 : %d Counts<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Interrupt Flag
    	value = VCNL4035X01_GET_PS_Interrupt();
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	CyDelay(2000);

    #endif

    #ifdef Arduino

        Serial.println("***************************************************");
    	delay(50);

    	Serial.println("*******************Low Power Mode******************");

    	//Delay of PS Measurement + other Circuit Delay
    	delay(50);

    	//Print the Proximity Data (IRED 1)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
    	Serial.print(">>>>>>>Proximity Data 1 : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Proximity Data (IRED 2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
    	Serial.print(">>>>>>>Proximity Data 2 : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Proximity Data (IRED 3)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
    	Serial.print(">>>>>>>Proximity Data 3 : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Interrupt Flag
    	value = VCNL4035X01_GET_PS_Interrupt();
    	Serial.print(">>>>>>>Interrupt Flag : 0b");
    	Serial.print(value,BIN);
    	Serial.println("<<<<<<<<");
    	delay(50);

    	Serial.println("***************************************************");
    	delay(50);

    	Serial.println("");
    	delay(50);

    	Serial.println("");

    	delay(2000);

    #endif
}

/*Print the output of the ALS Mode
 *Print_ALS_Mode(Byte ALS_IT)
 *Byte ALS_IT - Input Parameter:
 *
 * VCNL4035X01_ALS_IT_50MS
 * VCNL4035X01_ALS_IT_100MS
 * VCNL4035X01_ALS_IT_200MS
 * VCNL4035X01_ALS_IT_400MS
 * VCNL4035X01_ALS_IT_800MS
 */
void Print_ALS_Mode(Byte ALS_IT)
{
	Word value;
	int Delay;
	float Sensitivity;
	float Lux;

    #ifdef STM32F

        #define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"**********************ALS Mode********************* \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Get Delay for ALS Measurement
    	Delay = VCNL4035X01_GET_Delay(ALS_IT);

    	//Delay of IT ms + other Circuit Delay (~10ms)
    	HAL_Delay(Delay);

    	//Find Sensitivity
    	Sensitivity = VCNL4035X01_CAL_Sensitivity(ALS_IT);

    	//Print the Sensitivity
    	ftoa(Sensitivity,TransmitBuffer, 6);
    	sprintf(TransmitBuffer2,">>>>>>>Sensitivity : %s lx/Count<<<<<<<<\r\n",TransmitBuffer);
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Print the ALS Data in DEC
    	value = VCNL4035X01_GET_ALS_Data();
    	sprintf(TransmitBuffer,">>>>>>>ALS Data : %d Counts<<<<<<<<\r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Calculate Lux
    	Lux = VCNL4035X01_CAL_Lux(Sensitivity, value);

    	//Print the ALS Lux in DEC
    	ftoa(Lux,TransmitBuffer, 5);
    	sprintf(TransmitBuffer2,">>>>>>>ALS Lux : %s lx<<<<<<<<\r\n",TransmitBuffer);
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Print the Interrupt Flag (Except when Gesture Mode activated)
    	if(VCNL4035X01_GET_PS_Mode() != 3)
    	{
    		value = VCNL4035X01_GET_ALS_Interrupt();
    		sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);
    	}

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	HAL_Delay(2000);

    #endif

    #ifdef Cypress

    	#define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"**********************ALS Mode********************* \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

    	//Get Delay for ALS Measurement
    	Delay = VCNL4035X01_GET_Delay(ALS_IT);

    	//Delay of IT ms + other Circuit Delay (~10ms)
    	CyDelay(Delay);

    	//Find Sensitivity
    	Sensitivity = VCNL4035X01_CAL_Sensitivity(ALS_IT);

    	//Print the Sensitivity
    	ftoa(Sensitivity,TransmitBuffer, 6);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,">>>>>>>Sensitivity : %s lx/Count<<<<<<<<\r\n",TransmitBuffer);
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

    	//Print the ALS Data in DEC
    	value = VCNL4035X01_GET_ALS_Data();
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>ALS Data : %d Counts<<<<<<<<\r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Calculate Lux
    	Lux = VCNL4035X01_CAL_Lux(Sensitivity, value);

    	//Print the ALS Lux in DEC
    	ftoa(Lux,TransmitBuffer, 5);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,">>>>>>>ALS Lux : %s lx<<<<<<<<\r\n",TransmitBuffer);
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

    	//Print the Interrupt Flag (Except when Gesture Mode activated)
    	if(VCNL4035X01_GET_PS_Mode() != 3)
    	{
    		value = VCNL4035X01_GET_ALS_Interrupt();
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);
    	}

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	CyDelay(2000);

    #endif

    #ifdef Arduino

        Serial.println("***************************************************");
    	delay(50);

    	Serial.println("**********************ALS Mode*********************");
    	delay(50);

    	//Get Delay for ALS Measurement
    	Delay = VCNL4035X01_GET_Delay(ALS_IT);

    	//Delay of IT ms + other Circuit Delay (~10ms)
    	delay(Delay);

    	//Find Sensitivity
    	Sensitivity = VCNL4035X01_CAL_Sensitivity(ALS_IT);

    	//Print the Sensitivity
    	Serial.print(">>>>>>>Sensitivity : ");
    	Serial.print(Sensitivity,5);
    	Serial.println(" lx/count<<<<<<<<");
    	delay(50);

    	//Print the ALS Data in DEC
    	value = VCNL4035X01_GET_ALS_Data();
    	Serial.print(">>>>>>>ALS Data : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Calculate Lux
    	Lux = VCNL4035X01_CAL_Lux(Sensitivity, value);

    	//Print the ALS Lux
    	Serial.print(">>>>>>>ALS Lux : ");
    	Serial.print(Lux,5);
    	Serial.println(" lx<<<<<<<<");
    	delay(50);

    	//Print the Interrupt Flag (Except when Gesture Mode activated)
    	if(VCNL4035X01_GET_PS_Mode() != 3)
    	{
    		value = VCNL4035X01_GET_ALS_Interrupt();
            Serial.print(">>>>>>>Interrupt Flag : 0b");
    		Serial.print(value,BIN);
    		Serial.println("<<<<<<<<");
    		delay(50);
    	}

    	Serial.println("***************************************************");
    	delay(50);

    	Serial.println("");
    	delay(50);

    	Serial.println("");

    	delay(2000);

    #endif
}

/*Print the output of the White Mode
 *Print_White_Mode(Byte ALS_IT)
 *Byte ALS_IT - Input Parameter:
 *
 * VCNL4035X01_ALS_IT_50MS
 * VCNL4035X01_ALS_IT_100MS
 * VCNL4035X01_ALS_IT_200MS
 * VCNL4035X01_ALS_IT_400MS
 * VCNL4035X01_ALS_IT_800MS
 */
void Print_White_Mode(Byte ALS_IT)
{
	Word value;
	int Delay;

    #ifdef STM32F

        #define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*********************White Mode******************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Get Delay for White Measurement
    	Delay = VCNL4035X01_GET_Delay(ALS_IT);

    	//Delay of IT ms + other Circuit Delay (~10ms)
    	HAL_Delay(Delay);

    	//Print the White Data in DEC
    	value = VCNL4035X01_GET_White_Data();
    	sprintf(TransmitBuffer,">>>>>>>White Data : %d Counts<<<<<<<<\r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Interrupt Flag (Except when Gesture Mode activated)
    	if(VCNL4035X01_GET_PS_Mode() != 3)
    	{
    		value = VCNL4035X01_GET_ALS_Interrupt();
    		sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
    		CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    		HAL_Delay(50);
    	}

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	HAL_Delay(2000);

    #endif

    #ifdef Cypress

    	#define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*********************White Mode******************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

    	//Get Delay for White Measurement
    	Delay = VCNL4035X01_GET_Delay(ALS_IT);

    	//Delay of IT ms + other Circuit Delay (~10ms)
    	CyDelay(Delay);

    	//Print the White Data in DEC
    	value = VCNL4035X01_GET_White_Data();
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>White Data : %d Counts<<<<<<<<\r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Interrupt Flag (Except when Gesture Mode activated)
    	if(VCNL4035X01_GET_PS_Mode() != 3)
    	{
    		value = VCNL4035X01_GET_ALS_Interrupt();
            if(CDC_CDCIsReady() != 0u)
    	    {
        		sprintf(TransmitBuffer,">>>>>>>Interrupt Flag : 0x%x<<<<<<<< \r\n",value);
        		CDC_PutString(TransmitBuffer);
            }
    		CyDelay(50);
    	}

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	CyDelay(2000);

    #endif

    #ifdef Arduino

        Serial.println("***************************************************");
    	delay(50);

    	Serial.println("*********************White Mode********************");
    	delay(50);

    	//Get Delay for White Measurement
    	Delay = VCNL4035X01_GET_Delay(ALS_IT);

    	//Delay of IT ms + other Circuit Delay (~10ms)
    	delay(Delay);

    	//Print the White Data in DEC
    	value = VCNL4035X01_GET_White_Data();
    	Serial.print(">>>>>>>White Data : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts<<<<<<<<");
    	delay(50);

    	//Print the Interrupt Flag (Except when Gesture Mode activated)
    	if(VCNL4035X01_GET_PS_Mode() != 3)
    	{
    		value = VCNL4035X01_GET_ALS_Interrupt();
            Serial.print(">>>>>>>Interrupt Flag : 0b");
    		Serial.print(value,BIN);
    		Serial.println("<<<<<<<<");
    		delay(50);
    	}

    	Serial.println("***************************************************");
    	delay(50);

    	Serial.println("");
    	delay(50);

    	Serial.println("");

    	delay(2000);

    #endif
}

//Print the Register Value
void Print_Register_Settings()
{
	Word value;

    #ifdef STM32F

        #define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*****************Register Settings***************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Print ALS Mode
    	switch(VCNL4035X01_GET_ALS_Mode())
    	{
    		case 0 :	sprintf(TransmitBuffer2,">>>>>>>Mode : ALS Off/Shutdown \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 1 :	sprintf(TransmitBuffer2,">>>>>>>Mode : ALS Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 2 :	sprintf(TransmitBuffer2,">>>>>>>Mode : White Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    	}

    	sprintf(TransmitBuffer2,">>>>>>>>>>>>>>>>>>>>>>>ALS<<<<<<<<<<<<<<<<<<<<<<<<< \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Print the Reg 0x00 (ALS_CONF1 and ALS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_CONF_1);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x00 (ALS_CONF) Val : 0x%x \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x01 (ALS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDH);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x01 (ALS_THDH) Val : %d Counts\r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x02 (ALS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDL);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x02 (ALS_THDL) Val : %d Counts\r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Print PS Mode
    	switch(VCNL4035X01_GET_PS_Mode())
    	{
    		case 0 :	sprintf(TransmitBuffer2,">>>>>>>Mode : PS Off/Shutdown \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 1 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Auto/Self-Timed Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 2 :	sprintf(TransmitBuffer2,">>>>>>>Mode : AF/Continuous Forced Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 3 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Gesture Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    		case 4 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Low Power Mode \r\n");
    					CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    					HAL_Delay(50);
    					break;
    	}


    	sprintf(TransmitBuffer2,">>>>>>>>>>>>>>>>>>>>>>>>PS<<<<<<<<<<<<<<<<<<<<<<<<< \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	//Print the Reg 0x03 (PS_CONF1 and PS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_1);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x03 (PS_CONF1) Val : 0x%x \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x04 (PS_CONF3 and PS_MS)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_3);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x04 (PS_CONF3) Val : 0x%x \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x05 (PS_CANC)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CANC);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x05 (PS_CANC) Val : %d Counts \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x06 (PS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDL);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x06 (PS_THDL) Val : %d Counts \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	//Print the Reg 0x07 (PS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDH);
    	sprintf(TransmitBuffer,">>>>>>>Reg 0x07 (PS_THDH) Val : %d Counts \r\n",value);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2," \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	HAL_Delay(2000);

    #endif

    #ifdef Cypress

    	#define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*****************Register Settings***************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

    	//Print ALS Mode
        if(CDC_CDCIsReady() != 0u)
    	{
        	switch(VCNL4035X01_GET_ALS_Mode())
        	{
        		case 0 :	sprintf(TransmitBuffer2,">>>>>>>Mode : ALS Off/Shutdown \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 1 :	sprintf(TransmitBuffer2,">>>>>>>Mode : ALS Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 2 :	sprintf(TransmitBuffer2,">>>>>>>Mode : White Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        	}
        }
        CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,">>>>>>>>>>>>>>>>>>>>>>>ALS<<<<<<<<<<<<<<<<<<<<<<<<< \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

    	//Print the Reg 0x00 (ALS_CONF1 and ALS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_CONF_1);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x00 (ALS_CONF) Val : 0x%x \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x01 (ALS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDH);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x01 (ALS_THDH) Val : %d Counts\r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x02 (ALS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDL);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x02 (ALS_THDL) Val : %d Counts\r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

    	//Print PS Mode
        if(CDC_CDCIsReady() != 0u)
    	{
        	switch(VCNL4035X01_GET_PS_Mode())
        	{
        		case 0 :	sprintf(TransmitBuffer2,">>>>>>>Mode : PS Off/Shutdown \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 1 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Auto/Self-Timed Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 2 :	sprintf(TransmitBuffer2,">>>>>>>Mode : AF/Continuous Forced Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 3 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Gesture Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        		case 4 :	sprintf(TransmitBuffer2,">>>>>>>Mode : Low Power Mode \r\n");
        					CDC_PutString(TransmitBuffer2);
        					break;
        	}
        }
        CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,">>>>>>>>>>>>>>>>>>>>>>>>PS<<<<<<<<<<<<<<<<<<<<<<<<< \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

    	//Print the Reg 0x03 (PS_CONF1 and PS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_1);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x03 (PS_CONF1) Val : 0x%x \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x04 (PS_CONF3 and PS_MS)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_3);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x04 (PS_CONF3) Val : 0x%x \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x05 (PS_CANC)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CANC);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x05 (PS_CANC) Val : %d Counts \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x06 (PS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDL);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x06 (PS_THDL) Val : %d Counts \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

    	//Print the Reg 0x07 (PS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDH);
        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Reg 0x07 (PS_THDH) Val : %d Counts \r\n",value);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2," \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	CyDelay(2000);

    #endif

    #ifdef Arduino

        Serial.println("***************************************************");
    	delay(50);

    	Serial.println("*****************Register Settings*****************");
    	delay(50);

    	//Print ALS Mode
    	switch(VCNL4035X01_GET_ALS_Mode())
    	{
    		case 0 :	Serial.println(">>>>>>>Mode : ALS Off/Shutdown");
    					break;
    		case 1 :	Serial.println(">>>>>>>Mode : ALS Mode");
    					break;
    		case 2 :	Serial.println(">>>>>>>Mode : White Mode");
    					break;
    	}
    	delay(50);

    	Serial.println(">>>>>>>>>>>>>>>>>>>>>>>ALS<<<<<<<<<<<<<<<<<<<<<<<<<");
    	delay(50);

    	//Print the Reg 0x00 (ALS_CONF1 and ALS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_CONF_1);
    	Serial.print(">>>>>>>Reg 0x00 (ALS_CONF) Val : 0x");
    	Serial.println(value,HEX);
    	delay(50);

    	//Print the Reg 0x01 (ALS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDH);
    	Serial.print(">>>>>>>Reg 0x01 (ALS_THDH) Val : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts ");
    	delay(50);

    	//Print the Reg 0x02 (ALS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_ALS_THDL);
    	Serial.print(">>>>>>>Reg 0x02 (ALS_THDL) Val : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts ");
    	delay(50);

    	Serial.println("***************************************************");
    	delay(50);

    	//Print PS Mode
    	switch(VCNL4035X01_GET_PS_Mode())
    	{
    		case 0 :	Serial.println(">>>>>>>Mode : PS Off/Shutdown");
    					break;
    		case 1 :	Serial.println(">>>>>>>Mode : Auto/Self-Timed Mode");
    					break;
    		case 2 :	Serial.println(">>>>>>>Mode : AF/Continuous Forced Mode");
    					break;
    		case 3 :	Serial.println(">>>>>>>Mode : Gesture Mode");
    					break;
    		case 4 :	Serial.println(">>>>>>>Mode : Low Power Mode");
    					break;
    	}
    	delay(50);

    	Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>PS<<<<<<<<<<<<<<<<<<<<<<<<<");
    	delay(50);

    	//Print the Reg 0x03 (PS_CONF1 and PS_CONF2)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_1);
    	Serial.print(">>>>>>>Reg 0x03 (PS_CONF1) Val : 0x");
    	Serial.println(value,HEX);
    	delay(50);

    	//Print the Reg 0x04 (PS_CONF3 and PS_MS)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CONF_3);
    	Serial.print(">>>>>>>Reg 0x04 (PS_CONF3) Val : 0x");
    	Serial.println(value,HEX);
    	delay(50);

    	//Print the Reg 0x05 (PS_CANC)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_CANC);
    	Serial.print(">>>>>>>Reg 0x05 (PS_CANC) Val : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts ");
    	delay(50);

    	//Print the Reg 0x06 (PS_THDL)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDL);
    	Serial.print(">>>>>>>Reg 0x06 (PS_THDL) Val : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts ");
    	delay(50);

    	//Print the Reg 0x07 (PS_THDH)
    	value = VCNL4035X01_READ_Reg(VCNL4035X01_PS_THDH);
    	Serial.print(">>>>>>>Reg 0x07 (PS_THDH) Val : ");
    	Serial.print(value,DEC);
    	Serial.println(" Counts ");
    	delay(50);

    	Serial.println("***************************************************");
    	delay(50);

    	Serial.println("");
    	delay(50);

    	Serial.println("");

    	delay(2000);

    #endif
}

/*Print the variable in DEC for debugging
 *Print_Variable_DEC(Word Var)
 *Word Var - Input Parameter:
 *
 * Value between 0d0 and 0d65535
 */
void Print_Variable_DEC(Word Var)
{
    #ifdef STM32F

        #define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer,">>>>>>>Variable : 0d%d  \r\n",Var);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	HAL_Delay(2000);

    #endif

    #ifdef Cypress

    	#define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Variable : 0d%d  \r\n",Var);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	CyDelay(2000);

    #endif

    #ifdef Arduino

        Serial.println("***************************************************");
    	delay(50);

    	Serial.print(">>>>>>>Variable : 0d");
        Serial.println(Var,DEC);
    	delay(50);

    	Serial.println("***************************************************");

    	delay(2000);

    #endif
}

/*Print the variable in HEX for debugging
 *Print_Variable_HEX(Word Var)
 *Word Var - Input Parameter:
 *
 * Value between 0d0 and 0d65535
 */
void Print_Variable_HEX(Word Var)
{
    #ifdef STM32F

        #define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer,">>>>>>>Variable : 0x%x  \r\n",Var);
    	CDC_Transmit_FS(TransmitBuffer,strlen(TransmitBuffer));
    	HAL_Delay(50);

    	sprintf(TransmitBuffer2,"*************************************************** \r\n");
    	CDC_Transmit_FS(TransmitBuffer2,strlen(TransmitBuffer2));

    	HAL_Delay(2000);

    #endif

    #ifdef Cypress

    	#define TRANSMIT_BUFFER_SIZE  128
    	char   TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    	char   TransmitBuffer2[TRANSMIT_BUFFER_SIZE];

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer,">>>>>>>Variable : 0x%x  \r\n",Var);
        	CDC_PutString(TransmitBuffer);
        }
    	CyDelay(50);

        if(CDC_CDCIsReady() != 0u)
    	{
        	sprintf(TransmitBuffer2,"*************************************************** \r\n");
        	CDC_PutString(TransmitBuffer2);
        }

    	CyDelay(2000);

    #endif

    #ifdef Arduino

        Serial.println("***************************************************");
    	delay(50);

        Serial.print(">>>>>>>Variable : 0x");
        Serial.println(Var,HEX);
    	delay(50);

    	Serial.println("***************************************************");

    	delay(2000);

    #endif
}

//Reverses a string 'str' of length 'len'
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

//Converts a given integer x to string str[].
//d is the number of digits required in the output.
//If d is more than the number of digits in x,
//then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;

    while (x)
	{
		//Store and convert int to char (Valid for single digit)
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    //If number of digits required is more, then
    //add 0s at the beginning
    while (i < d) str[i++] = '0';

	//Reverse the string characters in the array str
    reverse(str, i);

	//Place the null character at the end of the array
    str[i] = '\0';

	//Return the position i
    return i;
}

//Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    //Extract integer part
    int ipart = (int)n;

    //Extract decimal part
    float fpart = n - (float)ipart;

    //Convert integer part to string and the function returns the position after the interger
    int i = intToStr(ipart, res, 0);

    //Check for display option after point
    if (afterpoint != 0)
	{
		//Add dot after the integer part
        res[i] = '.';

        //Multiply decimal part by 10^decimal point
        fpart = fpart* pow(10, afterpoint);

		//Convert decimal part to string
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}