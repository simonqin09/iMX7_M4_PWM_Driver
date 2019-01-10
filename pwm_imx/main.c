/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include "board.h"
#include "gpio_pins.h"
#include "gpio_imx.h"
#include "debug_console_imx.h"
#include "pwm_imx.h"

/*! @brief PWM period value. PWMO (Hz) = PCLK(Hz) / (period +2) */
#define PWM_PERIOD_VALUE		0X4000
#define PWM_DUTYCYCLE_NUM       3

#define GPIO_INTERRUPT         (1)
#define GPIO_POLLING           (0)
#define GPIO_DEBOUNCE_DELAY    (100000)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* button relevent variables */
#ifdef BOARD_GPIO_KEY_CONFIG
static volatile uint8_t button_pressed_flag;
#endif

/* PWM relevent variables */
volatile uint32_t pwmDutycycle[3] = {0X1000U, 0X2000U, 0X3000U};
volatile uint32_t pwmDutycycleFlag = 0U;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Initialize GPIO INT\POLLING controller.
 */
static void GPIO_Ctrl_InitKeyPin(uint32_t gpioMode)
{
#ifdef BOARD_GPIO_KEY_CONFIG
    if (GPIO_INTERRUPT == gpioMode)
    {
        /* GPIO module initialize, configure button as interrupt mode. */
        gpio_init_config_t keyInitConfig = {
            .pin = BOARD_GPIO_KEY_CONFIG->pin,
            .direction = gpioDigitalInput,
            .interruptMode = gpioIntFallingEdge,
        };
        GPIO_Init(BOARD_GPIO_KEY_CONFIG->base, &keyInitConfig);
        /* Enable interrupt. */
        NVIC_EnableIRQ(BOARD_GPIO_KEY_IRQ_NUM);
    }
    else
    {
        /* GPIO module initialize, configure button as GPIO functionality. */
        gpio_init_config_t keyInitConfig = {
            .pin = BOARD_GPIO_KEY_CONFIG->pin,
            .direction = gpioDigitalInput,
            .interruptMode = gpioNoIntmode,
        };
        GPIO_Init(BOARD_GPIO_KEY_CONFIG->base, &keyInitConfig);
    }
#endif
}

/*!
 * @brief Wait user to press key in INT\NOINT mode.
 */
static void GPIO_WaitKeyPressed(uint32_t gpioMode)
{
#ifdef BOARD_GPIO_KEY_CONFIG
    uint32_t i, j, debounce;

    if (GPIO_INTERRUPT == gpioMode)
    {
        do
        {
            debounce = 0;

            /* Clear the interrupt state, this operation is necessary, because the GPIO module maybe confuse
               the first rising edge as interrupt*/
            GPIO_ClearStatusFlag(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin);
            /* Enable GPIO pin interrupt */
            GPIO_SetPinIntMode(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin, true);

            /* Waitting for Key pressed. */
            while(button_pressed_flag == 0);
            button_pressed_flag = 0;

            for (i = 0; i < 3; i++)
            {
                /* Delay to wait key value stable. The cycle number should be changed
                 * according to M4 Core clock frequncy.
                 */
                for (j = 0 ; j < GPIO_DEBOUNCE_DELAY; j++)
                {
                    __NOP();
                }

                if (0 == GPIO_ReadPinInput(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin))
                {
                    debounce++;
                }
            }

            if (debounce > 2)
            {
                break;
            }
        } while (1);
    }
    else
    {
        /* Wait for Key Released. */
        do
        {
            debounce = 0;
            while (0 == GPIO_ReadPinInput(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin));

            for (i = 0; i < 3; i++)
            {
                /* Delay to wait key value stable. The cycle number should be changed
                 * according to M4 Core clock frequency.
                 */
                for (j = 0 ; j < GPIO_DEBOUNCE_DELAY; j++)
                {
                    __NOP();
                }

                if (1 == GPIO_ReadPinInput(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin))
                {
                    debounce++;
                }
            }

            if (debounce > 2)
            {
                break;
            }
        }
        while (1);

        /* Wait for Key Pressed. */
        do
        {
            debounce = 0;
            while (1 == GPIO_ReadPinInput(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin));

            for (i = 0; i < 3; i++)
            {
                /* Delay to wait key value stable. The cycle number should be changed
                 * according to M4 Core clock frequncy.
                 */
                for (j = 0 ; j < GPIO_DEBOUNCE_DELAY; j++)
                {
                    __NOP();
                }

                if (0 == GPIO_ReadPinInput(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin))
                {
                    debounce++;
                }
            }

            if (debounce > 2)
            {
                break;
            }
        }
        while (1);
    }
#else
        GETCHAR();
#endif
}


void BOARD_PWM_HANDLER(void)
{
    /* Gets interrupt kPWM_FIFOEmptyFlag */
    if(PWM_GetStatusFlags(BOARD_PWM_BASEADDR) & kPWM_FIFOEmptyFlag)
    {
        /* Clear kPWM_FIFOEmptyFlag */
        PWM_clearStatusFlags(BOARD_PWM_BASEADDR, kPWM_FIFOEmptyFlag);
    }
}

/*****************************************************************************
*
* Function Name: main
* Comments: PWM and GPIO module initialize, interrupt and IO operation.
*   This example include 2 step:
*   1)Configure PWM with default parameters and enable related interrupt.
*   2)Configure BUTTON1 as GPIO functionality
*   3)Click BUTTON1 to switch PWM output duty cycle
*
******************************************************************************/
int main(void)
{
    pwm_config_t pwmConfig;
    
    /* hardware initialiize, include RDC, IOMUX, Uart debug initialize */
    hardware_init();
    PRINTF("\n\r====================== PWM driver Example ========================\n\r");
    
    /*!
     * config->enableStopMode = false;
     * config->enableDozeMode = false;
     * config->enableWaitMode = false;
     * config->enableDebugMode = true;
     * config->clockSource = kPWM_LowFrequencyClock;
     * config->prescale = 0U;
     * config->outputConfig = kPWM_SetAtRolloverAndClearAtcomparison;
     * config->fifoWater = kPWM_FIFOWaterMark_3;
     * config->sampleRepeat = kPWM_EachSampleOnce;
     * config->byteSwap = kPWM_ByteNoSwap;
     * config->halfWordSwap = kPWM_HalfWordNoSwap;
     */
    PWM_GetDefaultConfig(&pwmConfig);

    /* Initialize PWM module */
    PWM_Init(BOARD_PWM_BASEADDR, &pwmConfig);

    /* Enable FIFO empty interrupt */
    PWM_EnableInterrupts(BOARD_PWM_BASEADDR, kPWM_FIFOEmptyInterruptEnable);

    /* Initial samples be written to the PWM Sample Register */
    PWM_SetSampleValue(BOARD_PWM_BASEADDR, pwmDutycycle[0]);

    /* Check and Clear interrupt status flags */
    if(PWM_GetStatusFlags(BOARD_PWM_BASEADDR))
    {
        PWM_clearStatusFlags(BOARD_PWM_BASEADDR, kPWM_FIFOEmptyFlag | kPWM_RolloverFlag | kPWM_CompareFlag | kPWM_FIFOWriteErrorFlag);
    }

    /* Write the period to the PWM Period Register */
    PWM_SetPeriodValue(BOARD_PWM_BASEADDR, PWM_PERIOD_VALUE);

    /* Set PWM Interrupt priority */
    NVIC_SetPriority(BOARD_PWM_IRQ_NUM, 3);

    /* Call core API to enable the IRQ. */
    NVIC_EnableIRQ(BOARD_PWM_IRQ_NUM);

    /* Initiate GPIO KEY as POLLING mode */
    GPIO_Ctrl_InitKeyPin(GPIO_POLLING);

    /* Start PWM Output */
    PWM_StartTimer(BOARD_PWM_BASEADDR);
    PRINTF("\n\r============== PWM Functionality Start ===============\n\r");
    
    while(true)
    {
    	GPIO_WaitKeyPressed(GPIO_POLLING);
    	PRINTF("\n\r============== Key Pressed ===============\n\r");
    	pwmDutycycleFlag = (pwmDutycycleFlag+1) % PWM_DUTYCYCLE_NUM;

    	switch (pwmDutycycleFlag)
    	{
    	case 0:
    		PWM_SetSampleValue(BOARD_PWM_BASEADDR, pwmDutycycle[0]);
    		break;
    	case 1:
    		PWM_SetSampleValue(BOARD_PWM_BASEADDR, pwmDutycycle[1]);
    		break;
    	case 2:
    		PWM_SetSampleValue(BOARD_PWM_BASEADDR, pwmDutycycle[2]);
    		break;
    	}

    	PRINTF("\n\r============== PWM Dutycycle Changed ===============\n\r");

    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
