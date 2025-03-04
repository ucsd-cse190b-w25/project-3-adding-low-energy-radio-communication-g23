/* USER CODE BEGIN Header */
  /**
    ******************************************************************************
    * @file           : main.c
    * @brief          : Main program body
    ******************************************************************************
    * @attention
    *
    * Licensed under BSD 3-Clause license
    ******************************************************************************
    */
  /* USER CODE END Header */

  /* Includes ------------------------------------------------------------------*/
  #include "ble.h"
  #include "i2c.h"
  #include "lsm6dsl.h"

  #include <stdlib.h>
  #include <stdint.h>
  #include <stdio.h>
  #include <string.h>
  #include <math.h>

  int dataAvailable = 0;
  SPI_HandleTypeDef hspi3;

  void SystemClock_Config(void);
  static void MX_GPIO_Init(void);
  static void MX_SPI3_Init(void);

  /**
    * @brief  The application entry point.
    * @retval int
    */

  int main(void){
    /* Reset peripherals and initialize system */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI3_Init();

    /* RESET BLE MODULE */
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

    ble_init();
    HAL_Delay(100);

    /* init I2C and Accelerometer */
    i2c_init();
    lsm6dsl_init();

    HAL_Delay(100);

	/* motion detection vars */
	int16_t prev_x = 1000000, prev_y = 1000000, prev_z = 1000000;
	int16_t cur_x, cur_y, cur_z;
	int lost_timer = -1;  // count when no movement is detected
	int THRESHOLD = 100;  // noise threshold
	int discoverable = 1; // 0 means its not discoverable, 1 is discoverable
	int moving = 1;

    while(1){
		catchBLE();
    	/* condition for init and when movement occurs */
    	if((discoverable && moving && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port, BLE_INT_Pin)) || moving){

			HAL_Delay(100);
			disconnectBLE(); // disconnect BLE

			HAL_Delay(100);
			setDiscoverability(0); // set to non discoverable
			HAL_Delay(100);

			discoverable = 0; // update to non discoverable
			moving = 0; // update to not moving
		}

    	/* post init: non discoverable or still */
		else {
			HAL_Delay(1000);

			lsm6dsl_read_xyz(&cur_x, &cur_y, &cur_z); // read accelerometer values

			/* condition to check for still */
			if (abs(cur_x - prev_x) <= THRESHOLD && abs(cur_y - prev_y) <= THRESHOLD && abs(cur_z - prev_z) <= THRESHOLD){
					 if (lost_timer < 0) {
							lost_timer = 0;
					 }
					 else {
							lost_timer++; // update seconds still
					 }

					 /* lost mode */
					  if (lost_timer >= 60 && discoverable == 0){
					    setDiscoverability(1); // set discoverable
						discoverable = 1; // set discoverable
						moving = 0; // set not moving
					  }

					  /* send log for how long it has been lost every 10 sec */
					  if (discoverable && (lost_timer % 10 == 0)){
						    char msg[50];
							snprintf(msg, sizeof(msg), "PrivTag AK has been missing for %d seconds", lost_timer);
							updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(msg), (uint8_t*)msg);

					  }
			}

		    /* airtag moving */
			else{
				lost_timer = -1; // reset timer
				moving = 1; // found
			}

			/* update accelerometer vals */
			prev_x = cur_x;
			prev_y = cur_y;
			prev_z = cur_z;

		}
     }
  }

  /**
    * @brief System Clock Configuration
    * @attention This changes the System clock frequency, make sure you reflect that change in your timer
    * @retval None
    */
  void SystemClock_Config(void)
  {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
      Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    // This lines changes system clock frequency
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
      Error_Handler();
    }
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
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : BLE_INT_Pin */
    GPIO_InitStruct.Pin = BLE_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
    GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : BLE_CS_Pin */
    GPIO_InitStruct.Pin = BLE_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
  }

  /* USER CODE BEGIN 4 */

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

  #ifdef  USE_FULL_ASSERT
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
