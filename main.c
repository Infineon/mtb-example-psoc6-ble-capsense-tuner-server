/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Tuning CapSense
*              over BLE - Server Example for ModusToolbox.
*
* Related Document: See Readme.md
*
*
********************************************************************************
* Copyright (2020), Cypress Semiconductor Corporation. All rights reserved.
********************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_capsense.h"
#include "cycfg_ble.h"
#include "tuner_ble_server.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define CAPSENSE_INTR_PRIORITY  (7u)
#define CY_ASSERT_FAILED        (0u)


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static cy_status initialize_capsense(void);
static void capsense_isr(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. This function performs
*  - initial setup of device
*  - initialize CapSense
*  - initialize ble for tuner communication
*  - scan touch input continuously.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_status status = CYRET_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,\
                                 CY_RETARGET_IO_BAUDRATE);

    /* Initialize the User LED */
    result |= cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED1,\
                               CYHAL_GPIO_DIR_OUTPUT,\
                               CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize CapSense block */
    status = initialize_capsense();

    /* Halt the CPU if CapSense initialization failed */
    if(CYRET_SUCCESS != status)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Initialize BLESS block */
    ble_capsense_tuner_init();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "\
           "Tuning CapSense over BLE - Server"\
           " ****************** \r\n\n");

    /* Start the initial CapSense scan */
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    for(;;)
    {
        /* Process the BLE stack events */
        ble_process_events();

        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Establishes synchronized operation between the CapSense
             * middleware and the CapSense Tuner tool.
             */
            Cy_CapSense_RunTuner(&cy_capsense_context);

            /* Start next scan */
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        }
    }
}


/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function does the following
*  - initializes the CapSense
*  - configure the CapSense interrupt.
*  - register callback functions to be used for tuner ble
*
*  Return:
*   - cy_status
*
*******************************************************************************/
static cy_status initialize_capsense(void)
{
    cy_status status = CYRET_SUCCESS;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_ISR_cfg =
    {
            .intrSrc = CYBSP_CSD_IRQ,
            .intrPriority = CAPSENSE_INTR_PRIORITY,
    };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

     if(CYRET_SUCCESS == status)
     {
         /* Initialize CapSense interrupt */
         Cy_SysInt_Init(&CapSense_ISR_cfg, capsense_isr);
         NVIC_ClearPendingIRQ(CapSense_ISR_cfg.intrSrc);
         NVIC_EnableIRQ(CapSense_ISR_cfg.intrSrc);

         /* Initialize the CapSense firmware modules. */
         status = Cy_CapSense_Enable(&cy_capsense_context);
     }

    /* Register tuner communication callback */
    cy_capsense_context.ptrCommonContext->ptrTunerSendCallback = tuner_send_callback;

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
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}


/* [] END OF FILE */