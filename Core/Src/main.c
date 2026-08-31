/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include "main.h"
#include "mcp2515.h"
#include "can_schema.h"
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

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t pulse_counter;
uint32_t fault_start_time;
uint8_t fault_was_active;
uint32_t clear_start_time;
uint8_t clearing_in_progress;
uint8_t power_cut_active;
uint32_t prev_pulse_count;
uint32_t prev_time;
float wheel_speed_pps;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Flags whether the current wheel-speed reading should be trusted.
 *
 * Wheel speed (linear) and gyro-Z (yaw rate) measure different physical
 * quantities, so this is not a complementary filter fusing two measurements
 * of the same signal. It's a plausibility check: high yaw rate indicates
 * cornering, during which a single wheel's speed diverges from vehicle
 * speed due to slip, making it a less trustworthy proxy for overall speed.
 *
 * Threshold is a placeholder pending real cornering data from the vehicle;
 * not empirically tuned here since that requires field data unavailable
 * on the bench.
 */
uint8_t get_slip_context(float gyro_z_dps)
{
    const float YAW_RATE_THRESHOLD_DPS = 30.0f;

    if (fabsf(gyro_z_dps) > YAW_RATE_THRESHOLD_DPS)
    {
        return CAN_SLIP_CONTEXT_UNRELIABLE;
    }
    else
    {
        return CAN_SLIP_CONTEXT_TRUSTWORTHY;
    }
}

CAN_Frame pack_safety_frame(uint8_t fault_active, uint8_t power_cut, uint8_t clearing, GPIO_PinState brake)
{
    CAN_Frame frame = { .id = CAN_ID_SAFETY, .dlc = 2 };
    frame.data[0] = 0;
    if (fault_active) frame.data[0] |= CAN_SAFETY_BIT_APPS_FAULT;
    if (power_cut)     frame.data[0] |= CAN_SAFETY_BIT_POWER_CUT;
    if (clearing)      frame.data[0] |= CAN_SAFETY_BIT_CLEARING;
    frame.data[1] = (brake == GPIO_PIN_RESET) ? 1 : 0;  // active-low: RESET means pressed
    return frame;
}

CAN_Frame pack_driver_input_frame(float apps1_pct, float apps2_pct, GPIO_PinState brake)
{
    CAN_Frame frame = { .id = CAN_ID_DRIVER_INPUT, .dlc = 3 };
    frame.data[0] = (uint8_t)apps1_pct;
    frame.data[1] = (uint8_t)apps2_pct;
    frame.data[2] = (brake == GPIO_PIN_RESET) ? 1 : 0;  // active-low: RESET means pressed
    return frame;
}

CAN_Frame pack_motion_frame(float wheel_speed, float gyro_dps, uint8_t slip_flag)
{
    CAN_Frame frame = { .id = CAN_ID_MOTION, .dlc = 5 };

    uint16_t speed_fixed = (uint16_t)wheel_speed;
    frame.data[0] = (speed_fixed >> 8) & 0xFF;
    frame.data[1] = speed_fixed & 0xFF;

    int16_t gyro_fixed = (int16_t)(gyro_dps * 100.0f);
    frame.data[2] = (gyro_fixed >> 8) & 0xFF;
    frame.data[3] = gyro_fixed & 0xFF;

    frame.data[4] = slip_flag;

    return frame;
}

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  char boot_msg[] = "Board booted, UART alive - FALLING EDGE TEST v2\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)boot_msg, sizeof(boot_msg) - 1, 100);

  uint8_t who_am_i_result;
  HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x75, I2C_MEMADD_SIZE_8BIT, &who_am_i_result, 1, 100);

  uint8_t wake_cmd = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, I2C_MEMADD_SIZE_8BIT, &wake_cmd, 1, 100);

  MCP2515_Handle can1 = { .csPort = GPIOC, .csPin = cs1_Pin };

  // Startup self-test: bring up the CAN controller and verify TX/RX via
  // internal loopback before entering the main loop. Retries a bounded
  // number of times to tolerate transient signal issues on the breadboard
  // prototyping platform; halts only if all attempts fail.
  uint8_t can_ready = 0;
  for (uint8_t boot_attempt = 0; boot_attempt < 3 && !can_ready; boot_attempt++)
  {
      can_ready = 1;
      can_ready &= MCP2515_Reset(&hspi3, &can1);
      can_ready &= MCP2515_SetBitTiming(&hspi3, &can1, 0x01, 0xAA, 0x05);  // 125kbps @ 8MHz osc
      can_ready &= MCP2515_SetMode(&hspi3, &can1, MCP2515_MODE_LOOPBACK);

      if (can_ready)
      {
          CAN_Frame tx_frame = { .id = 0x123, .dlc = 2, .data = {0xAB, 0xCD} };
          MCP2515_SendFrame(&hspi3, &can1, &tx_frame);
          HAL_Delay(5);

          CAN_Frame rx_frame;
          MCP2515_ReadFrame(&hspi3, &can1, &rx_frame);

          can_ready = (rx_frame.id == tx_frame.id && rx_frame.dlc == tx_frame.dlc &&
                       rx_frame.data[0] == tx_frame.data[0] && rx_frame.data[1] == tx_frame.data[1]);
      }
  }

  char diag_msg[64];
  int diag_len = snprintf(diag_msg, sizeof(diag_msg), "can_ready=%d\r\n", can_ready);
  HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, diag_len, 100);

  if (!can_ready)
  {
      Error_Handler();  // CAN self-test failed after retries; do not proceed with an unverified link
  }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t apps1_raw;
  uint32_t apps2_raw;
  GPIO_PinState brake_state;
  float apps1_pct;
  float apps2_pct;
  uint8_t gyro_z_raw_bytes[2];
  float gyro_z_dps;
  uint32_t current_time;
  uint32_t delta_count;
  uint32_t delta_time;
  uint32_t last_safety_send = 0;
  uint32_t last_driver_send = 0;
  uint32_t last_motion_send = 0;

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_ADC_Start(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 1);
    apps1_raw = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 1);
    apps2_raw = HAL_ADC_GetValue(&hadc1);

    brake_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

    apps1_pct = (apps1_raw / 4095.0) * 100;
    apps2_pct = (apps2_raw / 4095.0) * 100;

    if (fabsf(apps1_pct - apps2_pct) > 10)
    {
        if (fault_was_active == 0)
        {
            fault_start_time = HAL_GetTick();
            fault_was_active = 1;
            clearing_in_progress = 0;
        }
        else
        {
            if (HAL_GetTick() - fault_start_time > 100)
            {
                power_cut_active = 1;
            }
        }
    }
    else
    {
        fault_was_active = 0;

        if (power_cut_active == 1)
        {
            if (clearing_in_progress == 0)
            {
                clear_start_time = HAL_GetTick();
                clearing_in_progress = 1;
            }
            else
            {
                if (HAL_GetTick() - clear_start_time > 100)
                {
                    power_cut_active = 0;
                    clearing_in_progress = 0;
                }
            }
        }
    }
    HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x47, I2C_MEMADD_SIZE_8BIT, gyro_z_raw_bytes, 2, 100);
    int16_t gyro_z_raw = (gyro_z_raw_bytes[0] << 8) | gyro_z_raw_bytes[1];
    gyro_z_dps = gyro_z_raw / 131.0;

    current_time = HAL_GetTick();

    if (current_time - prev_time >= 100)
    {
        delta_count = pulse_counter - prev_pulse_count;
        delta_time = current_time - prev_time;

        wheel_speed_pps = (delta_count * 1000.0) / delta_time;

        prev_pulse_count = pulse_counter;
        prev_time = current_time;
    }

    uint8_t slip_context = get_slip_context(gyro_z_dps);

    // Safety frame: event-driven (fault state changed) OR periodic heartbeat
    static uint8_t prev_fault_state = 0;
    uint8_t current_fault_state = fault_was_active | (power_cut_active << 1) | (clearing_in_progress << 2);
    uint8_t safety_state_changed = (current_fault_state != prev_fault_state);

    if (safety_state_changed || (current_time - last_safety_send >= CAN_INTERVAL_SAFETY_MS))
    {
        CAN_Frame safety_frame = pack_safety_frame(fault_was_active, power_cut_active, clearing_in_progress, brake_state);
        MCP2515_SendFrame(&hspi3, &can1, &safety_frame);
        last_safety_send = current_time;
        prev_fault_state = current_fault_state;
    }

    if (current_time - last_driver_send >= CAN_INTERVAL_DRIVER_INPUT_MS)
    {
        CAN_Frame driver_frame = pack_driver_input_frame(apps1_pct, apps2_pct, brake_state);
        MCP2515_SendFrame(&hspi3, &can1, &driver_frame);
        last_driver_send = current_time;
    }

    if (current_time - last_motion_send >= CAN_INTERVAL_MOTION_MS)
    {
        CAN_Frame motion_frame = pack_motion_frame(wheel_speed_pps, gyro_z_dps, slip_context);
        MCP2515_SendFrame(&hspi3, &can1, &motion_frame);
        last_motion_send = current_time;
    }

    char uart_buf[128];
    int len = snprintf(uart_buf, sizeof(uart_buf),
        "APPS1:%.1f APPS2:%.1f BRAKE:%d FAULT:%d PCUT:%d WSPD:%.1f GYRO:%.2f SLIP:%d\r\n",
        apps1_pct, apps2_pct, (brake_state == GPIO_PIN_RESET) ? 1 : 0,
        fault_was_active, power_cut_active, wheel_speed_pps, gyro_z_dps, slip_context);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 100);
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, cs1_Pin|cs2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : cs1_Pin cs2_Pin */
  GPIO_InitStruct.Pin = cs1_Pin|cs2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    pulse_counter++;
    char msg[] = "PULSE DETECTED\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, 100);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
