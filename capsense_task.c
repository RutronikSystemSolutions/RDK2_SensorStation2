/*
 * capsense_task.c
 *
 *  Created on: Feb 2, 2023
 *      Author: Gintaras
 */

#include "capsense_task.h"
#include "cycfg_capsense.h"

#define CAPSENSE_INTR_PRIORITY      (7u)
#define EZI2C_INTR_PRIORITY         (6u)

static uint32_t initialize_capsense(void);
static void capsense_isr(void);
static void capsense_callback();
static void process_touch(void);

/*CapSense Task Handles*/
TaskHandle_t capsense_task_handle = NULL;
volatile bool capsense_scan_complete = false;
uint32_t slider_position = 0;

void capsense_task(void *param)
{
	(void) param;
	cy_rslt_t result;

    /*CapSense Initializations*/
    result = initialize_capsense();
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /* Initiate first scan */
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
	for(;;)
	{
		vTaskDelay(pdMS_TO_TICKS(20));

    	/*Check if the CSD slider scan is complete*/
        if (capsense_scan_complete)
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Process touch input */
            process_touch();

            /* Initiate next scan */
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

            capsense_scan_complete = false;
        }
	}
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configure the CapSense
*  interrupt.
*
*******************************************************************************/
static uint32_t initialize_capsense(void)
{
    uint32_t status = CYRET_SUCCESS;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_interrupt_config =
        {
            .intrSrc = CAPSENSE_IRQ,
            .intrPriority = CAPSENSE_INTR_PRIORITY,
        };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
    NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
    NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Assign a callback function to indicate end of CapSense scan. */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E, capsense_callback, &cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    return status;
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CAPSENSE_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: capsense_callback()
********************************************************************************
* Summary:
*  This function sets a flag to indicate end of a CapSense scan.
*
* Parameters:
*  cy_stc_active_scan_sns_t* : pointer to active sensor details.
*
*******************************************************************************/
void capsense_callback(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    capsense_scan_complete = true;
}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
*******************************************************************************/
static void process_touch(void)
{
    cy_stc_capsense_touch_t *slider_touch_info;
    uint16_t slider_pos;
    uint8_t slider_touch_status;
    static uint16_t slider_pos_prev;

    /* Get slider status */
    slider_touch_info = Cy_CapSense_GetTouchInfo(CY_CAPSENSE_LINEARSLIDER_WDGT_ID, &cy_capsense_context);
    slider_touch_status = slider_touch_info->numPosition;
    slider_pos = slider_touch_info->ptrPosition->x;

    /* Detect the new touch on slider */
    if ((0 != slider_touch_status) && (slider_pos != slider_pos_prev))
    {
    	slider_position = (slider_pos * 100) / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER_WDGT_ID].xResolution;
    }
}
