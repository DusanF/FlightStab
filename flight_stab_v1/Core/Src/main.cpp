/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "mpu.h"
#include "PIDlib.h"
extern "C" {
	#include "buzzer.h"
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum flighmode_type{
	FM_MAN,
	FM_STAB1,
	FM_STAB2
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_RB_SIZE 32	//rx ring buffer size

#define PID_FM1_ROLL_KP 0.1	//PIDs
#define PID_FM1_ROLL_KI 0.01
#define PID_FM1_ROLL_KD 0.1
#define PID_FM1_ROLL_WUM 100
#define PID_FM1_PITCH_KP 0.2
#define PID_FM1_PITCH_KI 0.02
#define PID_FM1_PITCH_KD 0.02
#define PID_FM1_PITCH_WUM 50
#define PID_FM1_YAW_KP 0.3
#define PID_FM1_YAW_KI 0.02
#define PID_FM1_YAW_KD 0.1
#define PID_FM1_YAW_WUM 50

#define PID_FM2_ROLL_KP 1
#define PID_FM2_ROLL_KI 0.08
#define PID_FM2_ROLL_KD 0.9
#define PID_FM2_ROLL_WUM 20
#define PID_FM2_PITCH_KP 1.5
#define PID_FM2_PITCH_KI 0.05
#define PID_FM2_PITCH_KD 0.25
#define PID_FM2_PITCH_WUM 25
#define PID_FM2_YAW_KP 2
#define PID_FM2_YAW_KI 0.1
#define PID_FM2_YAW_KD 0.6
#define PID_FM2_YAW_WUM 30

#define FLAP_STR 0.7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED_ON HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define BEEPER_ON HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET)
#define BEEPER_OFF HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET)
#define BEEPER_SET(__v__) HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, __v__)
#define LIMIT_RC(__v__) if(__v__ > 2000) __v__=2000; if(__v__ < 1000) __v__=1000;
#define LIMIT_NORM(__v__) if(__v__ > 1) __v__=1; if(__v__ < -1) __v__=-1;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


volatile unsigned char rx_rbuff[RX_RB_SIZE] = {0};
volatile uint8_t rx_rbuff_ptr = 0;
float rc_values[7] = {0, 0, 0, 0, 0, 0, 0};

volatile char byte_rdy=0, rc_new_rdy=0, mpu_ready=0;	//flags: new byte received, new RC values ready, new sensor data available

flighmode_type flightmode = FM_MAN;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){	//item in buzzer queue finished, process next
	buzzer_next();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {	//GPIO interrupt - new data from gyro available to be read
	if (GPIO_Pin == GYRO_INT_Pin)
		mpu_ready = 1;
}

void USART3_IRQHandler(void){	Usart IRQ, RC RX
    if (((USART3->ISR & USART_ISR_ORE) != 0U) && (((USART3->CR1 & USART_CR1_RXNEIE) != 0U)))
    	USART3->ICR = UART_CLEAR_OREF;

	if ( USART3->ISR & UART_IT_RXNE) {	//new byte received, add to ring buffer and set flag
		unsigned char tmp = (uint8_t)(USART3->RDR);
		rx_rbuff[rx_rbuff_ptr] = tmp;

		if(++rx_rbuff_ptr == RX_RB_SIZE)
			rx_rbuff_ptr = 0;

		byte_rdy = 1;
	}
	HAL_UART_IRQHandler(&huart3);
}

void ibus_parse(volatile uint8_t* rbuff, uint8_t rbuff_pos){	//try parsing data in ring buffer
	if((rbuff[rbuff_pos] != 0x20) || (rbuff[(rbuff_pos+1) & 0x1F] != 0x40))	//stop if header is not found
		return;

	uint8_t ptr;
	uint16_t chsum_calc = 0xFF9F;

	for(uint8_t i=2; i<30; i++){	//calculate checksum of data in buffer
		ptr = (rbuff_pos+i) & 0x1F;
		chsum_calc -= rbuff[ptr];
	}

	uint16_t chsum_recv = (rbuff[(rbuff_pos+31) & 0x1F] << 8) + rbuff[(rbuff_pos+30) & 0x1F];

	if(chsum_recv != chsum_calc)	//compare to received checksum
		return;

	rc_values[0] = ((float)((rbuff[(rbuff_pos+3) & 0x1F] << 8) + rbuff[(rbuff_pos+2) & 0x1F]) - 1500.0) / 500.0;	//data seems to be valid, accept and normalize (1000-2000) -> (-1,1)
	rc_values[1] = ((float)((rbuff[(rbuff_pos+5) & 0x1F] << 8) + rbuff[(rbuff_pos+4) & 0x1F]) - 1500.0) / 500.0;
	rc_values[2] = ((float)((rbuff[(rbuff_pos+7) & 0x1F] << 8) + rbuff[(rbuff_pos+6) & 0x1F]) - 1500.0) / 500.0;
	rc_values[3] = ((float)((rbuff[(rbuff_pos+9) & 0x1F] << 8) + rbuff[(rbuff_pos+8) & 0x1F]) - 1500.0) / 500.0;
	rc_values[4] = ((float)((rbuff[(rbuff_pos+11) & 0x1F] << 8) + rbuff[(rbuff_pos+10) & 0x1F]) - 1500.0) / 500.0;
	rc_values[5] = ((float)((rbuff[(rbuff_pos+13) & 0x1F] << 8) + rbuff[(rbuff_pos+12) & 0x1F]) - 1500.0) / 500.0;
	rc_values[6] = ((float)((rbuff[(rbuff_pos+15) & 0x1F] << 8) + rbuff[(rbuff_pos+14) & 0x1F]) - 1500.0) / 500.0;

	rc_new_rdy = 1;
}

void startup_tone(){
	LED_ON;
	buzzer_enqueue(BUZZER_ON, 10);
	buzzer_enqueue(BUZZER_OFF, 50);
	buzzer_enqueue(BUZZER_ON, 10);
	buzzer_enqueue(BUZZER_OFF, 50);
	buzzer_enqueue(BUZZER_ON, 10);
	buzzer_enqueue(BUZZER_OFF, 50);
	buzzer_enqueue(BUZZER_ON, 10);
	buzzer_enqueue(BUZZER_OFF, 50);
	buzzer_enqueue(BUZZER_ON, 10);
	buzzer_enqueue(BUZZER_OFF, 0);
	LED_OFF;
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	//start PWM out for servos
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  __HAL_UART_FLUSH_DRREGISTER(&huart3);



  startup_tone();

  Servo ser_ail_r(900, 2100, 1500, &(TIM3->CCR1));
  Servo ser_ail_l(900, 2100, 1500, &(TIM4->CCR3));
  Servo ser_ele(1000, 2000, 1511, &(TIM3->CCR2));
  Servo ser_rud(988, 1874, 1477, &(TIM4->CCR2));

  PID pid_roll;
  PID pid_pitch;
  PID pid_yaw;

  HAL_Delay(1000);	//wait for a while, let sensors start up (no need to rush, aircraft still moves after connecting battery)

  Mpu sensor(&hi2c1);	//initialize IMU
  sensor.mpu_set_power_cfg(0, POWER_DISABLE_TEMP, WAKEUP_FREQ_1_25HZ, CLKSRC_GYRO_X);
  sensor.mpu_set_lowpass(LPF_FREQ_98);
  sensor.mpu_set_gyro_range(GYRO_RANGE_2000_DEGS);
  sensor.mpu_set_accel_range(ACCEL_RANGE_8G);
  sensor.mpu_set_samplerate_div(4);
  sensor.mpu_set_interrupt(MPU_INT_HOLD | MPU_INT_CLEAR_READ_ANY, MPU_INT_SRC_DATA_READY);

  HAL_Delay(3200);	//wait some more, ESC startup tone generates movement
  buzzer_wait();

  LED_ON;
  buzzer_enqueue(BUZZER_ON, 50);
  buzzer_enqueue(BUZZER_OFF, 0);

  sensor.mpu_gyro_calibrate(128, GYRO_INT_GPIO_Port, GYRO_INT_Pin);	//calibrate gyro drift

  buzzer_enqueue(BUZZER_ON, 300);	//calibration done
  buzzer_enqueue(BUZZER_OFF, 20);
  buzzer_enqueue(BUZZER_ON, 100);
  buzzer_enqueue(BUZZER_OFF, 0);
  LED_OFF;

  __HAL_UART_FLUSH_DRREGISTER(&huart3);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);	//start receiving data from RC RX
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	if(byte_rdy){	//new data received, try to decode
	byte_rdy = 0;
	if(rx_rbuff[rx_rbuff_ptr] == 0x20)
		ibus_parse(rx_rbuff, rx_rbuff_ptr);
	}

	if(rc_new_rdy){	//new RC data ready
		rc_new_rdy = 0;
		if(rc_values[5] < -0.8){		//switch flight modes
			if(flightmode != FM_MAN){
				flightmode = FM_MAN;
				buzzer_enqueue(BUZZER_ON, 150);
				buzzer_enqueue(BUZZER_OFF, 0);
			}
		} else if(rc_values[5] < 0.8){
			if(flightmode != FM_STAB1){
				flightmode = FM_STAB1;
				pid_roll.setPidAll(PID_FM1_ROLL_KP, PID_FM1_ROLL_KI, PID_FM1_ROLL_KD ,PID_FM1_ROLL_WUM);
				pid_pitch.setPidAll(PID_FM1_PITCH_KP, PID_FM1_PITCH_KI, PID_FM1_PITCH_KD, PID_FM1_PITCH_WUM);
				pid_yaw.setPidAll(PID_FM1_YAW_KP, PID_FM1_YAW_KI, PID_FM1_YAW_KD, PID_FM1_YAW_WUM);
				buzzer_enqueue(BUZZER_ON, 150);
				buzzer_enqueue(BUZZER_OFF, 50);
				buzzer_enqueue(BUZZER_ON, 150);
				buzzer_enqueue(BUZZER_OFF, 300);
			}
		} else
			if(flightmode != FM_STAB2){
				flightmode = FM_STAB2;
				pid_roll.setPidAll(PID_FM2_ROLL_KP, PID_FM2_ROLL_KI, PID_FM2_ROLL_KD, PID_FM2_ROLL_WUM);
				pid_pitch.setPidAll(PID_FM2_PITCH_KP, PID_FM2_PITCH_KI, PID_FM2_PITCH_KD, PID_FM2_PITCH_WUM);
				pid_yaw.setPidAll(PID_FM2_YAW_KP, PID_FM2_YAW_KI, PID_FM2_YAW_KD, PID_FM2_YAW_WUM);
				buzzer_enqueue(BUZZER_ON, 150);
				buzzer_enqueue(BUZZER_OFF, 50);
				buzzer_enqueue(BUZZER_ON, 150);
				buzzer_enqueue(BUZZER_OFF, 50);
				buzzer_enqueue(BUZZER_ON, 150);
				buzzer_enqueue(BUZZER_OFF, 0);
			}


		if(flightmode == FM_MAN){	//no stab. needed, apply input to outputs
			if(rc_values[4] > 0.5)	//flaperon
				ser_ail_l.set_norm(-rc_values[0] - FLAP_STR);	//apply flaps
			else
				ser_ail_l.set_norm(-rc_values[0]);		//no flaps
			ser_ele.set_norm(rc_values[1]);
			ser_rud.set_norm(-rc_values[3]);
			if(rc_values[4] > 0.5)	//flaperon
				ser_ail_r.set_norm(-rc_values[0] + FLAP_STR);
			else
				ser_ail_r.set_norm(-rc_values[0]);
		}
	}

	if(mpu_ready){	//new gyro data available
		mpu_ready = 0;

		float gyro[3];
		sensor.mpu_read_gyro_norm(gyro);	//read nortmalized data


		if(flightmode != FM_MAN){	//loop PID regulators, apply outputs
			ser_ail_l.set_norm(-pid_roll.update(rc_values[0] + (gyro[1]/100.0)));
			ser_ele.set_norm(pid_pitch.update(rc_values[1] - (gyro[0]/100)));
			ser_rud.set_norm(pid_yaw.update(-rc_values[3] - (gyro[2]/100.0)));
			ser_ail_r.set_norm(-pid_roll.update(rc_values[0] + (gyro[1]/100.0)));
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
