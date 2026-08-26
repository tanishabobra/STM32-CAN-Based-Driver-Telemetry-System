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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void mcp2515_write(GPIO_TypeDef* csPort, uint16_t csPin, uint8_t addr, uint8_t data)
{
    uint8_t tx[3] = {0x02, addr, data};
    uint8_t dummy_rx[3];  // discarded, but keeps RX FIFO serviced

    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, tx, dummy_rx, 3, 100);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
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
  /* USER CODE BEGIN 2 */
  uint8_t who_am_i_result;
  HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x75, I2C_MEMADD_SIZE_8BIT, &who_am_i_result, 1, 100);

  uint8_t wake_cmd = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, I2C_MEMADD_SIZE_8BIT, &wake_cmd, 1, 100);

  uint8_t reset_cmd = 0xC0;
  uint8_t reset_dummy_rx;

  HAL_GPIO_WritePin(GPIOC, cs1_Pin, GPIO_PIN_RESET);  // CS low - select chip
  HAL_SPI_TransmitReceive(&hspi3, &reset_cmd, &reset_dummy_rx, 1, 100);
  HAL_GPIO_WritePin(GPIOC, cs1_Pin, GPIO_PIN_SET);    // CS high - deselect
  HAL_Delay(10);

  HAL_GPIO_WritePin(GPIOC, cs2_Pin, GPIO_PIN_RESET);  // CS low - select chip 2
  HAL_SPI_TransmitReceive(&hspi3, &reset_cmd, &reset_dummy_rx, 1, 100);
  HAL_GPIO_WritePin(GPIOC, cs2_Pin, GPIO_PIN_SET);    // CS high - deselect
  HAL_Delay(10);

  // ---- CANSTAT isolation test: only this runs from here down ----
  uint8_t tx_stat[3] = {0x03, 0x0E, 0xFF};
  uint8_t rx_stat[3] = {0};

  HAL_GPIO_WritePin(GPIOC, cs1_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef stat_status = HAL_SPI_TransmitReceive(&hspi3, tx_stat, rx_stat, 3, 100);
  HAL_GPIO_WritePin(GPIOC, cs1_Pin, GPIO_PIN_SET);

  volatile uint8_t canstat = rx_stat[2];
  volatile HAL_StatusTypeDef canstat_hal_status = stat_status;

  // Burst-write CNF3, CNF2, CNF1 with verify-and-retry (handles marginal breadboard signal)
  uint8_t cnf_write_success = 0;
  uint8_t cnf_retry_count = 0;
  const uint8_t MAX_CNF_RETRIES = 5;

  while (cnf_write_success == 0 && cnf_retry_count < MAX_CNF_RETRIES)
  {
      // Write CNF3, CNF2, CNF1 in one burst (address auto-increments)
      uint8_t tx_burst[5] = {0x02, 0x28, 0x05, 0xAA, 0x01};
      uint8_t rx_burst_dummy[5];

      HAL_GPIO_WritePin(GPIOC, cs1_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi3, tx_burst, rx_burst_dummy, 5, 100);
      HAL_GPIO_WritePin(GPIOC, cs1_Pin, GPIO_PIN_SET);

      // Read back to verify
      uint8_t tx_readback[5] = {0x03, 0x28, 0xFF, 0xFF, 0xFF};
      uint8_t rx_readback[5];

      HAL_GPIO_WritePin(GPIOC, cs1_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi3, tx_readback, rx_readback, 5, 100);
      HAL_GPIO_WritePin(GPIOC, cs1_Pin, GPIO_PIN_SET);

      // Check all three registers landed correctly
      if (rx_readback[2] == 0x05 && rx_readback[3] == 0xAA && rx_readback[4] == 0x01)
      {
          cnf_write_success = 1;
      }
      else
      {
          cnf_retry_count++;
      }
  }

  volatile uint8_t cnf_config_ok = cnf_write_success;
  volatile uint8_t cnf_retries_used = cnf_retry_count;

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : cs1_Pin cs2_Pin */
  GPIO_InitStruct.Pin = cs1_Pin|cs2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
