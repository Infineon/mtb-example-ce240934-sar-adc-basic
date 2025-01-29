/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for PSOC4 HVMS SAR ADC example for
*              ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include <stdio.h>

/*******************************************************************************
* Macros
********************************************************************************/
/* 12-bit SAR ADC resolution limit with hexadecimal */
#define RESOLUTION_LIMIT        (0xFFF)

/* SAR ADC channel to get ADC result */
#define TARGET_CHANNEL          (0ul)

/* SAR ADC interrupt priority */
#define INTR_PRIORITY    (2ul)

/* 500 milliseconds interval to put the ADC result on terminal software */
#define CONVERSION_INTERVAL     (500ul)

/* buffer size to store the sentence to put on terminal software */
#define BUFFER_SIZE             (128u)

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Global variable to store ADC result */
uint16_t g_channel_result = 0;

/* Flag to check if it enters End of Scan interrupt handler */
bool g_eos_flag = false;

/* SAR interrupt configuration */
const cy_stc_sysint_t sar_intr_config = {
    .intrSrc = SAR_IRQ,                        /* Source of interrupt signal */
    .intrPriority = INTR_PRIORITY              /* Interrupt priority */
};

/*******************************************************************************
* Function Name: Handle_Interrupt
********************************************************************************
* Summary:
*  Interrupt handler to get the ADC result and set the flag.
*
* Parameters:
*  handlerArg (unused)
*  event (unused)
*
* Return:
*  none
*
********************************************************************************/
void Handle_Interrupt(void)
{
    uint32_t intrStatus = Cy_SAR_GetInterruptStatus(SAR_HW);

    /* Check if the interrupt is caused by End of Scan */
    if(CY_SAR_INTR_EOS == (CY_SAR_INTR_EOS & intrStatus))
    {
        /* Get the ADC result as a signed 16-bit integer */
        g_channel_result = (RESOLUTION_LIMIT & Cy_SAR_GetResult16(SAR_HW, TARGET_CHANNEL));

        /* Set the flag that represents the handler is done */
        g_eos_flag = true;
    }

    Cy_SAR_ClearInterrupt(SAR_HW, CY_SAR_INTR_EOS);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function.
*
* Parameters:
*  none
*
* Return:
*  int
*
********************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_stc_scb_uart_context_t CYBSP_UART_context;
    char buffer[BUFFER_SIZE];

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Enable global interrupts */
    __enable_irq();

    result = Cy_SysInt_Init(&sar_intr_config, Handle_Interrupt);

    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Set the interrupt priority and  */
    NVIC_ClearPendingIRQ(sar_intr_config.intrSrc);
    NVIC_EnableIRQ(sar_intr_config.intrSrc);

    /* Deinitialize SAR ADC before initialize it */
    result = Cy_SAR_DeInit(SAR_HW, true);

    if(result != CY_SAR_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Set End of Scan interrupt mask */
    Cy_SAR_SetInterruptMask(SAR_HW, CY_SAR_INTR_EOS);

    /* Open SARMUX switch to use gpio pin that connects to external potentiometer or thermistor as a VPLUS pin  */
    Cy_SAR_SetAnalogSwitch(SAR_HW, CY_SAR_MUX_FW_P3_VPLUS, true);

    /* Initialize SAR ADC */
    result = Cy_SAR_Init(SAR_HW, &SAR_config);

    if(result != CY_SAR_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable SAR ADC */
    Cy_SAR_Enable(SAR_HW);

    /* Start continuous ADC */
    Cy_SAR_StartConvert(SAR_HW, CY_SAR_START_CONVERT_CONTINUOUS);

    for (;;)
    {
        /* 500 milliseconds interval */
        Cy_SysLib_Delay(CONVERSION_INTERVAL);

        /* check if End of Scan interrupt occurs */
        if(g_eos_flag == true)
        {
        g_eos_flag = false;

        /* Store sentence and print it on terminal software by UART */
        sprintf(buffer, "ADC conversion value for Potentiometer : %d \n\r", g_channel_result);
        Cy_SCB_UART_PutString(CYBSP_UART_HW, buffer);
        }
    }
}

/* [] END OF FILE */
