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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FOOT 100
#define FR 101
#define FL 102
#define BR 103
#define BL 104

#define ARM1 110
#define ADD_RIGHT 111
#define ADD_LEFT 112
#define ADD_LIFT 113
#define ADD_ROTATE 114

#define ARM2 120
#define ADD_MOVE 121
#define PILLER_RIGHT 122
#define PILLER_LEFT 123
#define ROGER 124

#define ENC_FOOT 200

#define ENC_ARM 210


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t enc_state[10]; //FR, BL, FL, BR, pillerR, addR, addL, pillerL, add_rotate, add_move
uint8_t limit_state[10]; //rogerT,rogerB,pillerL,pillerR,addL,addR,liftT,liftB,moveL,moveR
int tx_flag = 0;
int tx_state = FOOT;

uint8_t currentPWM[12]; //FR, BL, FL, BR, addR, addL, addLift, addRotate, addMove, pillerR, pillerL, roger
uint8_t targetPWM[12]; //FR, BL, FL, BR, addR, addL, addLift, addRotate, addMove, pillerR, pillerL, roger
uint8_t DIR[4];
const uint8_t PCoefficient = 0.1;

uint8_t controler[8] = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1){
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK){
    	switch(RxHeader.StdId){
    	case 0x300:
    		// 受信データ RxData[8] があると仮定
    		enc_state[0] = (uint16_t)RxData[0] | ((uint16_t)RxData[1] << 8);
    		enc_state[1] = (uint16_t)RxData[2] | ((uint16_t)RxData[3] << 8);
    		enc_state[2] = (uint16_t)RxData[4] | ((uint16_t)RxData[5] << 8);
    		enc_state[3] = (uint16_t)RxData[6] | ((uint16_t)RxData[7] << 8);
    		tx_flag = 1;
    		tx_state = ENC_FOOT;
    		break;

    	case 0x301:
    		enc_state[4] = (uint16_t)RxData[0] | ((uint16_t)RxData[1] << 8);
    		enc_state[5] = (uint16_t)RxData[2] | ((uint16_t)RxData[3] << 8);
    		enc_state[6] = (uint16_t)RxData[4] | ((uint16_t)RxData[5] << 8);
    		enc_state[7] = (uint16_t)RxData[6] | ((uint16_t)RxData[7] << 8);
    		tx_flag = 2;
    		tx_state = ENC_ARM;
    		break;

    	case 0x302:
    		enc_state[8] = (uint16_t)RxData[0] | ((uint16_t)RxData[1] << 8);
    		enc_state[9] = (uint16_t)RxData[6] | ((uint16_t)RxData[7] << 8);
    		tx_flag = 3;
    		tx_state = ENC_ARM2;
    		break;
    }
    }
}

void ARMPWM(uint8_t motor_number, uint8_t button, uint8_t LS, int16_t currentVel) {
    // デフォルトは停止。ボタンが離されたら直ちにPWMを0にする。
    uint8_t output = 0;

    // ボタンが押されており、かつリミットスイッチが「押されていない(!LS)」場合のみ出力を出す
    // LSが1(真)になった瞬間に、論理積により強制的に出力が0になります。
    if (button && !LS) {
        output = 99; // 暫定出力
    }

    switch (motor_number) {
        case ADD_RIGHT:
            targetPWM[4] = output;
            break;
        case ADD_LEFT:
            targetPWM[5] = output;
            break;
        case ADD_LIFT:
            targetPWM[6] = output;
            break;
        case ADD_ROTATE:
            targetPWM[7] = output;
            break;
        case ADD_MOVE:
            targetPWM[8] = output;
            break;
        case PILLER_RIGHT:
            targetPWM[9] = output;
            break;
        case PILELR_LEFT:
            targetPWM[10] = output;
            break;
        case ROGER:
            targetPWM[11] = output;
            break;
        default:
            break;
    }
}

uint8_t footPWM(uint8_t motor_number, uint8_t stickX, uint8_t stickY, int16_t currentVel) {
        // 1. スティック値を -127 ~ 127 の範囲に正規化
        int16_t vx = (int16_t)stickX - 128;
        int16_t vy = (int16_t)stickY - 128;

        // 2. X字配置オムニの運動学（Kinematics）に基づいたターゲット速度の計算
        // FR: vy - vx, FL: vy + vx, BR: vy + vx, BL: vy - vx
        int16_t targetVel = 0;
        switch (motor_number) {
            case FL: targetVel = vy + vx; break;
            case FR: targetVel = vy - vx; break;
            case BL: targetVel = vy - vx; break;
            case BR: targetVel = vy + vx; break;
            default: return 0;
        }

        // 3. P制御 (Proportional Control)
        // PWM = Kp * (Target - Current)
        // 本来 Kp は float であるべきですが、整数演算にする場合はスケーリングします
        float Kp = 0.5f; // 仮のゲイン。PCoefficientの代わりに。
        int16_t error = targetVel - currentVel;
        int16_t output = (int16_t)(Kp * error);

        // 4. 出力制限と絶対値化（PWMは0~255のため）
        // 方向（DIR）は別途 motorTransmit で処理することを想定
        if (output > 255) output = 255;
        if (output < -255) output = -255;

        return (uint8_t)abs(output);
}

void motorTransmit(){
	CAN_TxHeaderTypeDef* TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];

    if( 0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan) && HAL_GetTick() - pre_transmit >= 10 ){
        switch(tx_state){
        case ENC_FOOT:
    	TxHeader.StdId = 0x100;
        case ENC_ARM
        }
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 8;
        TxHeader.TransmitGlobalTime = DISABLE;

        for(int i = 0; i < 4 ; i++){
        	TxData[i] = currentPWM[i];
        }
        for(int i = 0; i < 4; i++){
        	TxData[i+4] = currentDIR[i];
        }

        pre_transmit = HAL_GetTick();

        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    }
}
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  updatePWM();
	  if(tx_flag){
	  mortorTransmit();
	  tx_flag = 0;
	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
