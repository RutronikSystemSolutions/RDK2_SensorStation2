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

/* Set baud rate to special 1093750 */
#define ARDU_BAUD_RATE       		1093750

/*Imported I2C Device Global Variables*/
extern cyhal_i2c_t I2C_scb3;
extern cyhal_i2c_cfg_t i2c_scb3_cfg;

/*Thermal Imaging Task Handles*/
TaskHandle_t thermal_imaging_task_handle = NULL;
extern SemaphoreHandle_t i2c_mutex;

/*The MLX90640 Data Storage*/
thermal_image_t mlx90640 =
{
		.emissivity = 0.95
};

/*Arduino UART object and configuration*/
cyhal_uart_t ardu_uart;

/*Function prototypes*/
void ResetDisplay(void);
cy_rslt_t ardu_uart_init(void);

void thermal_imaging_task(void *param)
{
	(void) param;
	cy_rslt_t result;
	int err = MLX90640_NO_ERROR;
	int status = MLX90640_NO_ERROR;
	uint16_t *eeMLX90640 = NULL;

	printf("thermal imaging task has started.\r\n");

	/*Reset the Display*/
	ResetDisplay();

    /*Initialize The Arduino UART*/
    result = ardu_uart_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("Could not initialize Arduino UART.\r\n");
    	CY_ASSERT(0);
    }

	/*Initialize the thermal imaging sensor*/
	MLX90640_I2CInit();
	MLX90640_I2CGeneralReset();
	memset(mlx90640.mlx90640Frame, 0x00, sizeof(mlx90640.mlx90640Frame));
	memset(mlx90640.mlx90640To, 0x00, sizeof(mlx90640.mlx90640To));
	memset(&mlx90640.mlx90640Config, 0x00, sizeof(mlx90640.mlx90640Config));

	/*Read the device ID*/
	err = MLX90640_I2CRead(MLX90640_ADDR, MLX_ID1_ADDR, 3, mlx90640.device_id);
	if(err != MLX90640_NO_ERROR)
	{
		printf("MLX9064x device read ID failed.\r\n");
		CY_ASSERT(0);
	}
    printf("MLX9064x device is online, the serial number is: " );
    printf("0x%X", mlx90640.device_id[0]);
    printf("%X", mlx90640.device_id[1]);
    printf("%X\r\n", mlx90640.device_id[2]);

	/*Allocate memory for EEPROM data storage*/
    eeMLX90640 = malloc(MLX_EEPROM_SIZE);
    if(eeMLX90640 == NULL)
    {
    	CY_ASSERT(0);
    }
    memset(eeMLX90640, 0x00, MLX_EEPROM_SIZE);

    /*Get device parameters - We only have to do this once*/
    status = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
    if (status != 0)
    {
    	printf("Failed to load system parameters.\r\n");
    }
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640.mlx90640Config);
    if (status != 0)
    {
    	printf("Parameter extraction failed.\r\n");
    }

    /*Free the EEPROM data storage*/
    free(eeMLX90640);
    eeMLX90640 = NULL;

    /*Set the refresh rate*/
    MLX90640_SetRefreshRate(MLX90640_ADDR, MLX_REFRESH_RATE);

	/*POR Delay*/
	vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));

	for(;;)
	{
		/*Read the data and do the math*/
		for (uint8_t i = 0 ; i < 2 ; i++)
		{
			/*Delay determined by the refresh rate*/
			vTaskDelay(pdMS_TO_TICKS(1000/MLX_DELAY_DIV));

		    status = MLX90640_GetFrameData(MLX90640_ADDR, mlx90640.mlx90640Frame);
		    if(status == 0)
		    {
		    	/*Subpage 0 data*/
		    	cyhal_gpio_toggle(LED1);
		    	mlx90640.subpage = status;
		    }
		    else if(status == 1)
		    {

		    	/*Subpage 1 data*/
		    	cyhal_gpio_toggle(LED1);
		    	mlx90640.subpage = status;
		    }
		    else
		    {
		    	/*Error*/
		    	cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_OFF);
		    	continue;
		    }

		    mlx90640.vdd = MLX90640_GetVdd(mlx90640.mlx90640Frame, &mlx90640.mlx90640Config);
		    mlx90640.Ta = MLX90640_GetTa(mlx90640.mlx90640Frame, &mlx90640.mlx90640Config);
		    mlx90640.tr = mlx90640.Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
		    MLX90640_CalculateTo(mlx90640.mlx90640Frame, &mlx90640.mlx90640Config, mlx90640.emissivity, mlx90640.tr, mlx90640.mlx90640To);

		    /*Draw the subpage 0*/
		    if(status == 0)
		    {

		    }
		    else if(status == 1)
			{

			}
		}

	}
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

cy_rslt_t ardu_uart_init(void)
{
	cy_rslt_t result;
	uint32_t actualbaud;

    /* Initialize the UART configuration structure */
    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits = 8,
        .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL,
        .rx_buffer_size = 0
    };

    /* Initialize the UART Block */
    result = cyhal_uart_init(&ardu_uart, ARDU_TX, ARDU_RX, NC, NC, NULL, &uart_config);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	result = cyhal_uart_set_baud(&ardu_uart, ARDU_BAUD_RATE, &actualbaud);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	/*Connect internal pull-up resistor*/
	cyhal_gpio_configure(ARDU_RX, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP);

	return result;
}

/*Reset Display Function*/
void ResetDisplay(void)
{
	cyhal_gpio_write(ARDU_IO8, false);
	vTaskDelay(pdMS_TO_TICKS(500));
	cyhal_gpio_write(ARDU_IO8, true);
	vTaskDelay(pdMS_TO_TICKS(3000));
}

