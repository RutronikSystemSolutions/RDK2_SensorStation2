/*
 * gesture_control_task.c
 *
 *  Created on: Jan 9, 2023
 *      Author: Gintaras
 */

#include "VCNL4035X01.h"
#include "VCNL4035X01_Prototypes.h"
#include "typedefinition.h"
#include "VCNL4035X01_Application_Library.h"
#include "gesture_control_task.h"

/*Gesture Control Task Handles*/
TaskHandle_t gesture_control_task_handle = NULL;

/*I2C SCB3 Device */
int I2C_Bus = 3;

Byte PS_IT;
int IRED_Channel;
int SEL_Offset;

/* Variables for Offset Value */
int CalibValue = 0;
int OffsetValue = 0;
int AverageCount = 10; //Change the average count to the needed number of offset measurement
int Max_Offset = 0;

/*Gesture Mode Initializing Function Prototype*/
void Gesture_Mode();

void gesture_control_task(void *param)
{
	(void) param;
	Word vcnl4035x01_ID = 0;

	printf("gesture control task has started.\r\n");

    /* Reset Sensor to default value */
	Reset_Sensor();

	/* Read the VCNL4035X01 ID */
	vcnl4035x01_ID = VCNL4035X01_GET_ID();
	if(vcnl4035x01_ID != 0x80)
	{
		printf("VCNL4035X01 sensor failure.\r\n");
		for(;;)
		{
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	/*Gesture Mode Initialize*/
	Gesture_Mode();

	for(;;)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

/* Gesture Mode Initialization Function */
void Gesture_Mode()
{
    //1.)Initialization
    //Disable the PS
    VCNL4035X01_SET_PS_SD(VCNL4035X01_PS_SD_DIS);
    //Setting up AF Mode (Needed for Gesture Mode)
    VCNL4035X01_SET_PS_AF(VCNL4035X01_PS_AF_EN);

    //2.) Setting up PS
    //PS_CONF1
    //PS_DUTY doesn't have to be set for Gesture Mode
    //Set the Persistence
    VCNL4035X01_SET_PS_PERS(VCNL4035X01_PS_PERS_1);
    //Set the Integration Time
    VCNL4035X01_SET_PS_IT(PS_IT);

    //PS_CONF2
    //Enable/Disable the Gesture Interrupt
    VCNL4035X01_SET_GESTURE_INT(VCNL4035X01_PS_GESTURE_INT_EN);
    //Disable the Gesture Mode (Enable only after offset measurement)
    //If the mode be enabled now then only one channel will be measured
    VCNL4035X01_SET_GESTURE_MODE(VCNL4035X01_PS_GESTURE_MODE_DIS);
    //Set the Gain
    VCNL4035X01_SET_PS_Gain(VCNL4035X01_PS_GAIN_2_STEP);
    //Set the Output Bit Size
    VCNL4035X01_SET_PS_HD(VCNL4035X01_PS_HD_16Bits);
    //Set the Sensitivity Mode
    VCNL4035X01_SET_PS_NS(VCNL4035X01_PS_NS_2STEP_MODE_X4);
    //Set the Interrupt
    VCNL4035X01_SET_PS_INT(VCNL4035X01_PS_INT_CLOSE_AWAY);

    //PS_CONF3
    //Enable/Disable Low Current
    VCNL4035X01_SET_PS_LED_I_LOW(VCNL4035X01_PS_I_LOW_DIS);
    //Select IRED input to be Driven by the Internal Driver (For Gesture Mode not so important since all 3 channels will be read)
    VCNL4035X01_SET_PS_IRED_select(IRED_Channel);
    //Enable/Disable Smart Persistence
    VCNL4035X01_SET_PS_SMART_PERS(VCNL4035X01_PS_SMART_PERS_DIS);
    //Set Interrupt to Normal/Logic Mode
    VCNL4035X01_SET_PS_MS(VCNL4035X01_PS_MS_NORMAL);
    //Enable/Disable Sunlight Cancellation
    VCNL4035X01_SET_PS_SC_EN(VCNL4035X01_PS_SC_EN);
    //PS_AF has been set during initialization
    //PS_TRIG must be set before reading the proximity data

    //PS_MS
    //Set the Sunlight Cancellation Current
    VCNL4035X01_SET_PS_SC_CUR(VCNL4035X01_PS_SC_CURx1);
    //Set the Sunlight Capability
    VCNL4035X01_SET_PS_SP(VCNL4035X01_PS_SPx1);
    //Set the Output of Sunlight Protect Mode
    VCNL4035X01_SET_PS_SPO(VCNL4035X01_PS_SPO_00);
    //Set the LED Current
    VCNL4035X01_SET_PS_LED_I(VCNL4035X01_PS_LED_I_100mA);

    //3.) Switch On the sensor
    VCNL4035X01_SET_PS_SD(VCNL4035X01_PS_SD_EN);
    //Delay of 10 ms needs to be changed depending on the API of the µ-controller of use
    CyDelay(10);

    //Clear Initial Interrupt
    VCNL4035X01_GET_PS_Interrupt();
    CyDelay(10);

	//4.) Threshold Setting and Offset Measurement
	//Calculate Offset for all of the 3 channels
	for(int i=0; i<AverageCount;i++)
	{
		//Channel 1
		//Select IRED input to be driven by the internal driver to calibrate
		VCNL4035X01_SET_PS_IRED_select(VCNL4035X01_PS_IRED_1);
		//Delay of 10 ms needs to be changed depending on the API of the µ-controller of use
		CyDelay(10);
		//Enable trigger to start offset measurement
		VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);
		//Delay of 10 ms needs to be changed depending on the API of the µ-controller of use
		CyDelay(10);
		//Read offset data channel 1 and add as calibration value for offset
		CalibValue += VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_1);
		//Delay of 10 ms needs to be changed depending on the API of the µ-controller of use
		CyDelay(10);

		//Channel 2
		//Select IRED input to be driven by the internal driver to calibrate
		VCNL4035X01_SET_PS_IRED_select(VCNL4035X01_PS_IRED_2);
		//Delay of 10 ms needs to be changed depending on the API of the µ-controller of use
		CyDelay(10);
		//Enable trigger to start offset measurement
		VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);
		//Delay of 10 ms needs to be changed depending on the API of the µ-controller of use
		CyDelay(10);
		//Read offset data channel 2 and add as calibration value for offset
		CalibValue += VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_2);
		//Delay of 10 ms needs to be changed depending on the API of the µ-controller of use
		CyDelay(10);

		//Channel 3
		//Select IRED input to be driven by the internal driver to calibrate
		VCNL4035X01_SET_PS_IRED_select(VCNL4035X01_PS_IRED_3);
		//Delay of 10 ms needs to be changed depending on the API of the µ-controller of use
		CyDelay(10);
		//Enable trigger to start offset measurement
		VCNL4035X01_SET_PS_TRIG(VCNL4035X01_PS_TRIG_EN);
		//Delay of 10 ms needs to be changed depending on the API of the µ-controller of use
		CyDelay(10);
		//Read offset data channel 3 and add as calibration value for offset
		CalibValue += VCNL4035X01_READ_Reg(VCNL4035X01_PS_DATA_3);
		//Delay of 10 ms needs to be changed depending on the API of the µ-controller of use
		CyDelay(10);
	}

	//Calculate the average of the offset measurement from the 3 channels
	CalibValue /= 3*AverageCount;

	//Check whether the offset value is too high to be set in the cancellation register
	// 1T max offset value for the cancellation register is 2048
	if((PS_IT==VCNL4035X01_PS_IT_1T)||(PS_IT==VCNL4035X01_PS_IT_1_5T))Max_Offset = 2048;
	// 2T max offset value for the cancellation register is 4096
	if((PS_IT==VCNL4035X01_PS_IT_2T)||(PS_IT==VCNL4035X01_PS_IT_2_5T)||(PS_IT==VCNL4035X01_PS_IT_3T)||(PS_IT==VCNL4035X01_PS_IT_3_5T))Max_Offset = 4096;
	// 4T max offset value for the cancellation register is 8192
	if(PS_IT==VCNL4035X01_PS_IT_4T)Max_Offset = 8192;
	// 8T max offset value for the cancellation register is 16384
	if(PS_IT==VCNL4035X01_PS_IT_8T)Max_Offset = 16384;

	if(CalibValue>Max_Offset)
	{
		//Perform Offset Measurement
		if(SEL_Offset == 0) OffsetValue = 0;
		if(SEL_Offset == 1) OffsetValue = CalibValue;
		//Set Cancellation register to eliminate offset
		VCNL4035X01_SET_PS_CANC(OffsetValue);
		//Set Low Threshold
		VCNL4035X01_SET_PS_LowThreshold(3000 + OffsetValue);
		//set High Threshold
		VCNL4035X01_SET_PS_HighThreshold(5000 + OffsetValue);
	}
	else
	{
		//Perform Offset Measurement
		if(SEL_Offset == 0) OffsetValue = 0;
		if(SEL_Offset == 1) OffsetValue = CalibValue;
		//Set Cancellation register to eliminate offset
		VCNL4035X01_SET_PS_CANC(OffsetValue);
		//Set Low Threshold
		VCNL4035X01_SET_PS_LowThreshold(3000);
		//set High Threshold
		VCNL4035X01_SET_PS_HighThreshold(5000);
	}

    //5.) Enable the Gesture Mode
    //Enable/Disable the Gesture Mode
    VCNL4035X01_SET_GESTURE_MODE(VCNL4035X01_PS_GESTURE_MODE_EN);

    //while (I2C_1_MasterStatus()!=I2C_1_MODE_COMPLETE_XFER){}
    //CyDelay(1000);
}
