/**
 * @file bsp.c
 * @brief BSP：系统时钟、芯片外设配置、采样硬件启动。
 */
#include "bsp.h"
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "fdcan.h"
#include "adc.h"
#include "tim.h"
#include "spi.h"
#include "usart.h"

/* 常规组 DMA 循环缓冲，温度等慢任务直接读取。 */
uint16_t adc1_buff[2];
uint16_t adc2_buff[4];

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configur
	e the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
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

void BspInit(void)
{
    if (HAL_Init() != HAL_OK) Error_Handler();
    SystemClock_Config();

    /* 在采样中断启动前开启 CPU 周期计数，供辨识检查实际回调间隔。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_FDCAN1_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_TIM1_Init();
    MX_SPI3_Init();
    MX_TIM3_Init();
    MX_USART3_UART_Init();
}

/* 只准备 ADC；TIM1 触发由控制初始化阶段在 PWM 封波状态下启动。 */
void BspStartAdcSampling(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK ||
        HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    /* 关闭完成通知，保留 DMA 搬运、错误中断和 ADC 注入整组 JEOS 通知。 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buff, 2) != HAL_OK) Error_Handler();
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC);
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buff, 4) != HAL_OK) Error_Handler();
    __HAL_DMA_DISABLE_IT(hadc2.DMA_Handle, DMA_IT_HT | DMA_IT_TC);
}
