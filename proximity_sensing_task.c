/*
 * proximity_sensing_task.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Gintaras
 */

#include "proximity_sensing_task.h"

/*Proximity Sensing Task Handles*/
TaskHandle_t proximity_sensing_task_handle = NULL;

void proximity_sensing_task(void *param)
{
	(void) param;

	for(;;)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
