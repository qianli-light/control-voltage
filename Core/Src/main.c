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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_OLED.h"
#include "OLED.h"
#include "arm_math.h"

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

/* USER CODE BEGIN PV */
#define upper_limit                           8399.0f
#define lower_limit                           0.0f
#define BUCK_limit                            3780.0f
#define Middle_limit1                         3990.0f
#define Middle_limit2                         4410.0f
#define BOOST_limit                           8399.0f
#define BUCK_fix_pulse                        3864.0f
#define BOOST_fix_pulse                       294.0f
#define BUCK_max_pulse                        4199.0f
#define BOOST_min_pulse                       0.0f
#define software_step_size                    42.0f
#define software_start_digital_setpoint       4200.0f


uint16_t ADC_value[32];//2个通道，16个数据取平均滤波

const uint8_t buff_size=16;

float32_t phy_setpoint=0.0f;
float32_t phy_measurement[2];
float32_t calc_measurement[2];
float32_t calc_setpoint=0;
float32_t pid_memory[3];
float32_t debug_value=0;

const float32_t voltage_gain=1.0;
const float32_t phy_calc_conv=voltage_gain/(float32_t)buff_size;
const float32_t Ts=5e-5f;

arm_pid_instance_f32 pid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  calc_setpoint=phy_setpoint/phy_calc_conv;
  pid.Kp=0;
  pid.Ki=0;
  pid.Kd=0;
  pid.A0=pid.Kp+pid.Ki*Ts+pid.Kd/Ts;
  pid.A1=-pid.Kp-2.0f*pid.Kd/Ts;
  pid.A2=pid.Kd/Ts;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  OLED_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  arm_pid_init_f32(&pid,1);

  HAL_ADCEx_Calibration_Start(&hadc1,ADC_DIFFERENTIAL_ENDED);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_value,sizeof(ADC_value)/sizeof(uint16_t));

  HAL_TIM_Base_Start(&htim1);
  software_start();
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {




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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


uint32_t PID_Compute(const float32_t error) {
  arm_copy_f32(pid.state,pid_memory,3);
  const float32_t output = arm_pid_f32(&pid, error);
  debug_value=output;

  if (output > upper_limit)
  {
    arm_copy_f32(pid_memory,pid.state,3);
    return upper_limit;
  }
  if (output < lower_limit)
  {
    arm_copy_f32(pid_memory,pid.state,3);
    return lower_limit;
  }
  return (uint32_t)output;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance==TIM4)
  {
    static uint16_t out;
    calc_measurement[0]=ADC_value[0]+ADC_value[2]+ADC_value[4]+ADC_value[6]+ADC_value[8]+ADC_value[10]+ADC_value[12]+ADC_value[14]
                    +ADC_value[16]+ADC_value[18]+ADC_value[20]+ADC_value[22]+ADC_value[24]+ADC_value[26]+ADC_value[28]+ADC_value[30];
    //  calc_measurement[1]=ADC_value[1]+ADC_value[3]+ADC_value[5]+ADC_value[7]+ADC_value[9]+ADC_value[11]+ADC_value[13]+ADC_value[15]
    //     +ADC_value[17]+ADC_value[19]+ADC_value[21]+ADC_value[23]+ADC_value[25]+ADC_value[27]+ADC_value[29]+ADC_value[31];
    out=PID_Compute(calc_setpoint-calc_measurement[0]);
    if (out<=BUCK_limit)
    {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,out);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,BOOST_min_pulse);
    }
    else if (out>BUCK_limit  &&  out<=Middle_limit1)
    {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,out-126);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,BOOST_fix_pulse);
    }
    else if (out>Middle_limit1  &&  out<=Middle_limit2)
    {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,BUCK_fix_pulse);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,out-3654);
    }
    else if (out>Middle_limit2  &&  out<=BOOST_limit)
    {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,BUCK_max_pulse);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,out-4242);
    }


  }
}
void software_start(void) {
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  for (int i=0;i<=software_start_digital_setpoint;i+=software_step_size)
  {
    if (i<=BUCK_limit)
    {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,i);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,BOOST_min_pulse);
    }
    else if (i>BUCK_limit  &&  i<=Middle_limit1)
    {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,i-126);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,BOOST_fix_pulse);
    }
    else if (i>Middle_limit1  &&  i<=Middle_limit2)
    {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,BUCK_fix_pulse);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,i-3654);
    }
    else if (i>Middle_limit2  &&  i<=BOOST_limit)
    {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,BUCK_max_pulse);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,i-4242);
    }
    HAL_Delay(1);
  }





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
