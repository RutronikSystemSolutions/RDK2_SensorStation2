/*
 * thermal_imaging_task.c
 *
 *  Created on: Jan 2, 2023
 *      Author: Gintaras
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "thermal_imaging_task.h"
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"

/*Imported I2C Device Global Variables*/
extern cyhal_i2c_t I2C_scb3;
extern cyhal_i2c_cfg_t i2c_scb3_cfg;

TaskHandle_t thermal_imaging_task_handle = NULL;
extern SemaphoreHandle_t i2c_mutex;

float mlx90640To[768];
paramsMLX90640 mlx90640;

void thermal_imaging_task(void *param)
{
	(void) param;
	int err = MLX90640_NO_ERROR;
	int status;
	uint16_t *eeMLX90640 = NULL;

	printf("thermal imaging task has started.\r\n");

	/*Initialize the thermal imaging sensor*/
	MLX90640_I2CInit();
	MLX90640_I2CGeneralReset();

	/*Read the device ID*/
	uint16_t device_id[3] = {0};
	err = MLX90640_I2CRead(MLX90640_ADDR, 0x2407, 3, device_id);
	if(err != MLX90640_NO_ERROR)
	{
		printf("MLX9064x device read ID failed.\r\n");
		CY_ASSERT(0);
	}
    printf("MLX9064x device is online, the serial number is: " );
    printf("0x%X", device_id[0]);
    printf("%X", device_id[1]);
    printf("%X\r\n", device_id[2]);

	/*Allocate memory for eeprom storage*/
    eeMLX90640 = malloc(832*2);
    if(eeMLX90640 == NULL)
    {
    	CY_ASSERT(0);
    }
    memset(eeMLX90640, 0x00, 832*2);

    /*Get device parameters - We only have to do this once*/
    status = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
    if (status != 0)
    {
    	printf("Failed to load system parameters.\r\n");
    }
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status != 0)
    {
    	printf("Parameter extraction failed.\r\n");
    }

    /*Free the allocated memory*/
    free(eeMLX90640);


	for(;;)
	{}
}

void MLX90640_I2CInit(void)
{
	return;
}

int MLX90640_I2CGeneralReset(void)
{
	int ret = MLX90640_NO_ERROR;
	cy_rslt_t result;
	uint8_t data = 0x06;

    /*Write 0x06 to 0x00 I2C address*/
	result = cyhal_i2c_master_write( &I2C_scb3, 0x00, &data, 1, 10, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	return MLX90640_I2C_NACK_ERROR;
    }

	return ret;
}

void MLX90640_I2CFreqSet(int freq)
{
	cy_rslt_t result;

	xSemaphoreTake(i2c_mutex, portMAX_DELAY);

	/*Deinitialize I2C Master*/
	cyhal_i2c_free(&I2C_scb3);

    /*Initialize I2C Master*/
    result = cyhal_i2c_init(&I2C_scb3, ARDU_SDA, ARDU_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*Set the frequency of the I2C, the parameter "freq" is in Hz*/
    i2c_scb3_cfg.frequencyhal_hz = (uint32_t)(freq);
    result = cyhal_i2c_configure(&I2C_scb3, &i2c_scb3_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    xSemaphoreGive(i2c_mutex);
}

int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data)
{
	int ret = MLX90640_NO_ERROR;
	uint8_t i2c_data[4];
	cy_rslt_t result;

	/*MSByte address*/
	i2c_data[0] = (writeAddress >> 8) & 0xFF;
	/*LSByte address*/
	i2c_data[1] = writeAddress & 0xFF;
	/*MSByte data*/
	i2c_data[2] = (data >> 8) & 0xFF;
	/*LSByte data*/
	i2c_data[3] = data & 0xFF;

    /*Write data to I2C*/
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)slaveAddr, i2c_data, 4, 10, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	xSemaphoreGive(i2c_mutex);
    	return MLX90640_I2C_NACK_ERROR;
    }

    xSemaphoreGive(i2c_mutex);
	return ret;
}

int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
	int ret = MLX90640_NO_ERROR;
	uint8_t i2c_data[2];
	cy_rslt_t result;
	uint32_t cnt = 0;

	/*MSByte address*/
	i2c_data[0] = (startAddress >> 8) & 0xFF;
	/*LSByte address*/
	i2c_data[1] = startAddress & 0xFF;

    /*Write the address to I2C*/
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)slaveAddr, i2c_data, 2, 10, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	xSemaphoreGive(i2c_mutex);
    	return MLX90640_I2C_NACK_ERROR;
    }

    /*Read the data*/
	result = (int8_t)cyhal_i2c_master_read(&I2C_scb3,(uint16_t)slaveAddr, (uint8_t *)data, nMemAddressRead*2, 100, true);
	if (result != CY_RSLT_SUCCESS)
	{
		xSemaphoreGive(i2c_mutex);
		 return MLX90640_I2C_NACK_ERROR;
	}

	xSemaphoreGive(i2c_mutex);

	/*Swap bytes in every data address */
	for(cnt = 0; cnt < nMemAddressRead; cnt++)
	{
		data[cnt] = (data[cnt]>>8) | (data[cnt]<<8);
	}

	return ret;
}

