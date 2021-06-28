/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId fdekadTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
void        StartDefaultTask(void const* argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of fdekadTask */
    osThreadDef(fdekadTask, StartDefaultTask, osPriorityNormal, 0, 512);
    fdekadTaskHandle = osThreadCreate(osThread(fdekadTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM            = 1;
    RCC_OscInitStruct.PLL.PLLN            = 16;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA,
                      DEK0_NC_P7_17_Pin | DEK1_NO_p7_8_Pin | DEK2_NO_p7_9_Pin | DEK3_NO_p7_10_Pin | DEK4_NO_p7_11_Pin |
                          DEK5_NO_p7_12_Pin | DEK6_NO_p7_13_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LD3_Pin | REL_NO_DEK0_p5_6_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : DEK0_NC_P7_17_Pin DEK1_NO_p7_8_Pin DEK2_NO_p7_9_Pin DEK3_NO_p7_10_Pin
                             DEK4_NO_p7_11_Pin DEK5_NO_p7_12_Pin DEK6_NO_p7_13_Pin */
    GPIO_InitStruct.Pin = DEK0_NC_P7_17_Pin | DEK1_NO_p7_8_Pin | DEK2_NO_p7_9_Pin | DEK3_NO_p7_10_Pin |
                          DEK4_NO_p7_11_Pin | DEK5_NO_p7_12_Pin | DEK6_NO_p7_13_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : VCP_TX_Pin */
    GPIO_InitStruct.Pin       = VCP_TX_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : DEK_manual_Pin DEK_pulse_inp_p1_Pin DEK_reset_sw_Pin */
    GPIO_InitStruct.Pin  = DEK_manual_Pin | DEK_pulse_inp_p1_Pin | DEK_reset_sw_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : DEK_reset_input_Pin */
    GPIO_InitStruct.Pin  = DEK_reset_input_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(DEK_reset_input_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : VCP_RX_Pin */
    GPIO_InitStruct.Pin       = VCP_RX_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
    HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LD3_Pin REL_NO_DEK0_p5_6_Pin */
    GPIO_InitStruct.Pin   = LD3_Pin | REL_NO_DEK0_p5_6_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

#define SWITCH_AND_INPUT_PINS (DEK_manual_Pin | DEK_pulse_inp_p1_Pin | DEK_reset_sw_Pin | DEK_reset_input_Pin)

/// Dekade states
typedef enum {
    DEK1,
    DEK2,
    DEK3,
    DEK4,
    DEK5,
    DEK6,
    DEK7,
    DEK8,
    DEK9,
    DEK0,
    DEK_END,
} OutputState_e;

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the fdekadTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const* argument)
{
    /* USER CODE BEGIN 5 */
// #define SHOW_STATE
#define LED_TOGGLE_TICKS 5000
#define PULSE_HIGH_DEKAD_RESET_TICKS 3000

    TickType_t ticks_toggle = xTaskGetTickCount();

    uint32_t curr_pin_state = SWITCH_AND_INPUT_PINS;
    uint32_t prev_pin_state = SWITCH_AND_INPUT_PINS;
    uint32_t changed_pins   = 0;

    bool manual_pressed    = false;
    bool manual_changed    = false;
    bool pulse_inp_high    = false;
    bool pulse_inp_changed = false;
    bool reset_sw_pressed  = false;
    bool reset_sw_changed  = false;
    bool reset_inp_high    = false;
    bool reset_inp_changed = false;

    OutputState_e output_state = DEK0;
    /* Infinite loop */
    for (;;) {
        TickType_t ticks_now = xTaskGetTickCount();
        TickType_t elapsed   = ticks_now - ticks_toggle; // elapsed: duration

        if (elapsed >= LED_TOGGLE_TICKS) {
            ticks_toggle = ticks_now;
#ifdef SHOW_STATE
            for (uint8_t i = 0; i <= output_state; i++) {
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
                osDelay(150);
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                osDelay(150);
            }
#else
            //           HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
#endif
        }
        // Reset changed flags
        manual_changed    = false;
        pulse_inp_changed = false;
        reset_sw_changed  = false;
        reset_inp_changed = false;
        // Read and handle input pins
        curr_pin_state = GPIOA->IDR & SWITCH_AND_INPUT_PINS;
        changed_pins   = curr_pin_state ^ prev_pin_state;
        if (changed_pins) {
            // De-bounce
            osDelay(100);
            uint32_t tmp_pin_state = GPIOA->IDR & SWITCH_AND_INPUT_PINS;
            if (curr_pin_state == tmp_pin_state) {
                prev_pin_state = curr_pin_state & SWITCH_AND_INPUT_PINS;
                // some pin has changed!
                if (changed_pins & DEK_manual_Pin) {
                    manual_changed = true;
                    if (curr_pin_state & DEK_manual_Pin) {
                        // low when pressed...
                        manual_pressed = false;
                    } else {
                        manual_pressed = true;
                    }
                }
                if (changed_pins & DEK_pulse_inp_p1_Pin) {
                    pulse_inp_changed = true;
                    if (curr_pin_state & DEK_pulse_inp_p1_Pin) {
                        pulse_inp_high = true;
                    } else {
                        pulse_inp_high = false;
                    }
                }
                if (changed_pins & DEK_reset_sw_Pin) {
                    reset_sw_changed = true;
                    if (curr_pin_state & DEK_reset_sw_Pin) {
                        // low when pressed...
                        reset_sw_pressed = false;
                    } else {
                        reset_sw_pressed = true;
                    }
                }
                if (changed_pins & DEK_reset_input_Pin) {
                    reset_inp_changed = true;
                    if (curr_pin_state & DEK_reset_input_Pin) {
                        reset_inp_high = true;
                    } else {
                        reset_inp_high = false;
                    }
                }
            }
        }
        if (reset_inp_high) {
            (void) reset_inp_changed;
            // reset dekad
            if (output_state != DEK0) {
#warning check if we can set state DEK0 directly
                // Not at DEK0, next state
                output_state += 1;
            }
            osDelay(30);
        } else {
            // Check for positive flank on pulse input...
            if (pulse_inp_changed && pulse_inp_high) {
                // Show the change
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
                // HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            }
            // Check if impulse has been held/pressed
            if (!pulse_inp_changed && pulse_inp_high) {
                // nothing here
            }
            // Check for negative flank on pulse input...
            if (pulse_inp_changed && !pulse_inp_high) {
                // Next state
                output_state += 1;
                // nothing here
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
            }
        }
        // Check for positive flank on pulse and manual inputs...
        if (manual_changed && manual_pressed) {
            // Next state
            output_state += 1;
            // Show the change
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
            // HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        }
        // Check if impulse or manual has been held/pressed
        if (!manual_changed && manual_pressed) {
            // nothing here
        }
        // Check for negative flank on pulse and manual inputs...
        if (manual_changed && !manual_pressed) {
            // nothing here
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        }

        // Check for positive flank on manual input...
        if (reset_sw_changed && reset_sw_pressed) {
            // close REL_NC_p5_6_Pin
            output_state = DEK0;
        }

        // Check for output state overflow
        if (output_state >= DEK_END) {
            output_state = DEK1;
        }

        switch (output_state) {
        case DEK1:
            HAL_GPIO_WritePin(REL_NO_DEK0_p5_6_GPIO_Port, REL_NO_DEK0_p5_6_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port,
                              DEK2_NO_p7_9_Pin | DEK3_NO_p7_10_Pin | DEK4_NO_p7_11_Pin | DEK5_NO_p7_12_Pin |
                                  DEK6_NO_p7_13_Pin,
                              GPIO_PIN_RESET);
            HAL_GPIO_WritePin(REL_NO_DEK0_p5_6_GPIO_Port, DEK0_NC_P7_17_Pin, GPIO_PIN_SET);
            osDelay(10);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK1_NO_p7_8_Pin, GPIO_PIN_SET);
            break;
        case DEK2:
            HAL_GPIO_WritePin(REL_NO_DEK0_p5_6_GPIO_Port, REL_NO_DEK0_p5_6_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port,
                              DEK1_NO_p7_8_Pin | DEK3_NO_p7_10_Pin | DEK4_NO_p7_11_Pin | DEK5_NO_p7_12_Pin |
                                  DEK6_NO_p7_13_Pin,
                              GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK0_NC_P7_17_Pin, GPIO_PIN_SET);
            osDelay(10);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK2_NO_p7_9_Pin, GPIO_PIN_SET);
            break;
        case DEK3:
            HAL_GPIO_WritePin(REL_NO_DEK0_p5_6_GPIO_Port, REL_NO_DEK0_p5_6_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port,
                              DEK1_NO_p7_8_Pin | DEK2_NO_p7_9_Pin | DEK4_NO_p7_11_Pin | DEK5_NO_p7_12_Pin |
                                  DEK6_NO_p7_13_Pin,
                              GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK0_NC_P7_17_Pin, GPIO_PIN_SET);
            osDelay(10);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK3_NO_p7_10_Pin, GPIO_PIN_SET);
            break;
        case DEK4:
            HAL_GPIO_WritePin(REL_NO_DEK0_p5_6_GPIO_Port, REL_NO_DEK0_p5_6_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port,
                              DEK1_NO_p7_8_Pin | DEK2_NO_p7_9_Pin | DEK3_NO_p7_10_Pin | DEK5_NO_p7_12_Pin |
                                  DEK6_NO_p7_13_Pin,
                              GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK0_NC_P7_17_Pin, GPIO_PIN_SET);
            osDelay(10);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK4_NO_p7_11_Pin, GPIO_PIN_SET);
            break;
        case DEK5:
            HAL_GPIO_WritePin(REL_NO_DEK0_p5_6_GPIO_Port, REL_NO_DEK0_p5_6_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port,
                              DEK1_NO_p7_8_Pin | DEK2_NO_p7_9_Pin | DEK3_NO_p7_10_Pin | DEK4_NO_p7_11_Pin |
                                  DEK6_NO_p7_13_Pin,
                              GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK0_NC_P7_17_Pin, GPIO_PIN_SET);
            osDelay(10);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK5_NO_p7_12_Pin, GPIO_PIN_SET);
            break;
        case DEK6:
            HAL_GPIO_WritePin(REL_NO_DEK0_p5_6_GPIO_Port, REL_NO_DEK0_p5_6_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port,
                              DEK1_NO_p7_8_Pin | DEK2_NO_p7_9_Pin | DEK3_NO_p7_10_Pin | DEK4_NO_p7_11_Pin |
                                  DEK5_NO_p7_12_Pin,
                              GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK0_NC_P7_17_Pin, GPIO_PIN_SET);
            osDelay(10);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK6_NO_p7_13_Pin, GPIO_PIN_SET);
            break;
        case DEK7:
            // Intentional fallthrough
        case DEK8:
            // Intentional fallthrough
        case DEK9:
            HAL_GPIO_WritePin(REL_NO_DEK0_p5_6_GPIO_Port, REL_NO_DEK0_p5_6_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port,
                              DEK1_NO_p7_8_Pin | DEK2_NO_p7_9_Pin | DEK3_NO_p7_10_Pin | DEK4_NO_p7_11_Pin |
                                  DEK5_NO_p7_12_Pin | DEK6_NO_p7_13_Pin,
                              GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port, DEK0_NC_P7_17_Pin, GPIO_PIN_SET);
            osDelay(10);
            break;
        case DEK0:
            // Intentional fallthrough
        default:
            HAL_GPIO_WritePin(REL_NO_DEK0_p5_6_GPIO_Port, REL_NO_DEK0_p5_6_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DEK0_NC_P7_17_GPIO_Port,
                              DEK0_NC_P7_17_Pin | DEK1_NO_p7_8_Pin | DEK2_NO_p7_9_Pin | DEK3_NO_p7_10_Pin |
                                  DEK4_NO_p7_11_Pin | DEK5_NO_p7_12_Pin | DEK6_NO_p7_13_Pin,
                              GPIO_PIN_RESET);
            osDelay(10);
            break;
        }
        osDelay(10);
    }
    /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
