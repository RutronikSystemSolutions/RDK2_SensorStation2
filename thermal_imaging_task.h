/*
 * thermal_imaging_task.h
 *
 *  Created on: Jan 2, 2023
 *      Author: Gintaras
 */

#ifndef THERMAL_IMAGING_TASK_H_
#define THERMAL_IMAGING_TASK_H_

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define MLX90640_ADDR	0x33
#define MLX90641_ADDR	0x33

extern TaskHandle_t thermal_imaging_task_handle;

void thermal_imaging_task(void *param);

#endif /* THERMAL_IMAGING_TASK_H_ */
