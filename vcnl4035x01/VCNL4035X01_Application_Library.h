/*
 * VCNL4035X01_Application_Library.h
 *
 * Created  : 9 November 2020
 * Modified : 6 May 2021
 * Author   : HWanyusof
 * Version	: 1.0
 */

#include "typedefinition.h"

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
float VCNL4035X01_CAL_Sensitivity(Byte ALS_IT);

/*Calculate the ALS (Lux)
 *VCNL4035X01_CAL_Lux(float Sensitivity, float Count)
 *float Sensitivity - output from VCNL4035X01_CAL_Sensitivity(Byte ALS_IT)
 *float Count - output from VCNL4035X01_GET_ALS_Data()
 */
float VCNL4035X01_CAL_Lux(float Sensitivity, float Count);


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
int VCNL4035X01_GET_Delay(Byte ALS_IT);

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
int VCNL4035X01_GET_PS_Mode();

/*Get ALS Mode
 *returns the ALS mode status of the sensor as follows:
 *
 * 0 - ALS Shutdown Mode
 * 1 - ALS Mode
 * 2 - White Channel Mode
 *
 */
int VCNL4035X01_GET_ALS_Mode();

//Reset the Sensor to the default value
void Reset_Sensor();

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
void Print_All(Byte ALS_IT);

//Print the output of the Auto Mode / Self-Timed Mode
void Print_Auto_Mode();

//Print the output of the Active Force Mode / Continuous Forced Mode
void Print_AF_Mode();

//Print the output of the Gesture Mode
void Print_Gesture_Mode();

//Print the output of the Low Power Mode
void Print_Low_Power_Mode();

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
void Print_ALS_Mode(Byte ALS_IT);

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
void Print_White_Mode(Byte ALS_IT);

//Print the Register Value
void Print_Register_Settings();

/*Print the variable in DEC for debugging
 *Print_Variable_DEC(Word Var)
 *Word Var - Input Parameter:
 *
 * Value between 0d0 and 0d65535
 */
void Print_Variable_DEC(Word Var);

/*Print the variable in HEX for debugging
 *Print_Variable_HEX(Word Var)
 *Word Var - Input Parameter:
 *
 * Value between 0d0 and 0d65535
 */
void Print_Variable_HEX(Word Var);

/*Reverses a string 'str' of length 'len'
 *reverse(char* str, int len)
 *char* str - Array pointer to be reversed
 *int len - Length of the array
 */
void reverse(char* str, int len);

/*Converts a given integer x to string str[].
 *intToStr(int x, char str[], int d)
 *int x - floating-point number to be converted to a string (Both the integer part as well as fraction/decimal point part)
 *char str[] - output string of the floating-point number in the form of array character
 *int d - number of decimal point (the integer part always = 0, the fraction/decimal point part = int afterpoint from ftoa() input)
 */
int intToStr(int x, char str[], int d);

/*Converts a floating-point/double number to string.
 *ftoa(float n, char* res, int afterpoint)
 *float n - floating-point number to be converted to a string
 *char* res - pointer to output string of the floating-point number in the form of array character
 *int d - number of decimal point
 */
void ftoa(float n, char* res, int afterpoint);
