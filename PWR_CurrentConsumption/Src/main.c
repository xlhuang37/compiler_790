/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    PWR/PWR_CurrentConsumption/Src/main.c
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32G4xx PWR HAL API to enter
  *          and exit the Low Power Run mode.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_TOGGLE_DELAY         100

#define ACTIVE_MS   100u
#define SLEEP_MS    50u
#define INNER_ITERS 80000u
#define OUTER_ITERS 200u
#define FFT_N      128u
#define FFT_LOG2N  7u



#define FIR_TAPS      32u
#define EEG_SAMPLES   256u

// h[n] = 1/FIR_TAPS
static float fir_coeffs[FIR_TAPS];

// Simple buffers for one EEG window
static float eeg_in[EEG_SAMPLES];
static float eeg_out[EEG_SAMPLES];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

__IO uint32_t UserButtonStatus = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Decrease(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

// EEG
static void FIR_Init(void)
{
    float c = 1.0f / (float)FIR_TAPS;
    for (uint32_t i = 0; i < FIR_TAPS; i++)
    {
        fir_coeffs[i] = c;
    }

    for (uint32_t n = 0; n < EEG_SAMPLES; n++)
    {
        float slow   = (float)(n & 0x3F) * 0.01f;
        float fast   = ((n & 0x1) ? 0.5f : -0.5f);
        eeg_in[n]    = slow + fast;
        eeg_out[n]   = 0.0f;
    }
}

void FIR_Apply(const float *input, float *output, uint32_t length)
{
    for (uint32_t n = 0; n < length; n++)
    {
        float acc = 0.0f;

        for (uint32_t k = 0; k < FIR_TAPS; k++)
        {
            if (n >= k)
            {
                acc += fir_coeffs[k] * input[n - k];
            }
        }

        output[n] = acc;
    }
}

// FFT
typedef struct
{
    float real[FFT_N];
    float imag[FFT_N];
} FFTBuffer;

static float twiddle_real[FFT_N / 2];
static float twiddle_imag[FFT_N / 2];

__attribute__((noinline))
void FFT_Compute(FFTBuffer *buf);


static void FFT_InitTwiddles(void)
{
    for (uint32_t k = 0; k < FFT_N / 2; k++)
    {
        float angle = -2.0f * (float)M_PI * (float)k / (float)FFT_N;
        twiddle_real[k] = cosf(angle);
        twiddle_imag[k] = sinf(angle);
    }
}

static uint16_t bit_reverse(uint16_t x, uint16_t log2n)
{
    uint16_t n = 0;
    for (uint16_t i = 0; i < log2n; i++)
    {
        n = (uint16_t)((n << 1) | (x & 1u));
        x >>= 1;
    }
    return n;
}

__attribute__((noinline))
void FFT_Compute(FFTBuffer *buf)
{
    for (uint16_t i = 0; i < FFT_N; i++)
    {
        uint16_t j = bit_reverse(i, FFT_LOG2N);
        if (j > i)
        {
            float tmp_r = buf->real[i];
            float tmp_i = buf->imag[i];

            buf->real[i] = buf->real[j];
            buf->imag[i] = buf->imag[j];

            buf->real[j] = tmp_r;
            buf->imag[j] = tmp_i;
        }
    }

    for (uint16_t stage = 1; stage <= FFT_LOG2N; stage++)
    {
        uint16_t m      = (uint16_t)(1u << stage);
        uint16_t m2     = (uint16_t)(m >> 1);
        uint16_t step   = (uint16_t)(FFT_N / m);

        for (uint16_t k = 0; k < m2; k++)
        {
            float wr = twiddle_real[k * step];
            float wi = twiddle_imag[k * step];

            for (uint16_t j = k; j < FFT_N; j = (uint16_t)(j + m))
            {
                uint16_t t = (uint16_t)(j + m2);

                float ur = buf->real[j];
                float ui = buf->imag[j];

                float vr = buf->real[t];
                float vi = buf->imag[t];

                float tr = vr * wr - vi * wi;
                float ti = vr * wi + vi * wr;

                buf->real[j] = ur + tr;
                buf->imag[j] = ui + ti;

                buf->real[t] = ur - tr;
                buf->imag[t] = ui - ti;
            }
        }
    }
}


// Kalman Filter
typedef struct
{
    float err_measure;
    float err_estimate;
    float q;
    float current_estimate;
    float last_estimate;
    float kalman_gain;
} SimpleKalmanFilter;

void SimpleKalmanFilter_Init(SimpleKalmanFilter *kf,
                             float mea_e,
                             float est_e,
                             float q)
{
    kf->err_measure      = mea_e;
    kf->err_estimate     = est_e;
    kf->q                = q;
    kf->current_estimate = 0.0f;
    kf->last_estimate    = 0.0f;
    kf->kalman_gain      = 0.0f;
}

float SimpleKalmanFilter_UpdateEstimate(SimpleKalmanFilter *kf, float mea)
{
    kf->kalman_gain = kf->err_estimate / (kf->err_estimate + kf->err_measure);

    kf->current_estimate =
        kf->last_estimate + kf->kalman_gain * (mea - kf->last_estimate);

    kf->err_estimate =
        (1.0f - kf->kalman_gain) * kf->err_estimate
        + fabsf(kf->last_estimate - kf->current_estimate) * kf->q;

    kf->last_estimate = kf->current_estimate;

    return kf->current_estimate;
}

void SimpleKalmanFilter_SetMeasurementError(SimpleKalmanFilter *kf, float mea_e)
{
    kf->err_measure = mea_e;
}

void SimpleKalmanFilter_SetEstimateError(SimpleKalmanFilter *kf, float est_e)
{
    kf->err_estimate = est_e;
}

void SimpleKalmanFilter_SetProcessNoise(SimpleKalmanFilter *kf, float q)
{
    kf->q = q;
}

float SimpleKalmanFilter_GetKalmanGain(const SimpleKalmanFilter *kf)
{
    return kf->kalman_gain;
}

float SimpleKalmanFilter_GetEstimateError(const SimpleKalmanFilter *kf)
{
    return kf->err_estimate;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config_HighPerf();
    SimpleKalmanFilter kf;
    SimpleKalmanFilter_Init(&kf,
                            2.0f,
                            2.0f,
                            0.01f);

    volatile float sink = 0.0f;

    // Kalman Filter
//    for (int i = 0; i < 1000000; i++)
//    {
//        float measurement = 1.0f + 0.1f * (float)(i & 0xF);
//        float estimate = SimpleKalmanFilter_UpdateEstimate(&kf, measurement);
//        sink = estimate;
//    }
//    while (1)
//    {
//        __WFI();  // Wait For Interrupt → Sleep mode between interrupts
//    }

    uint32_t counter = 0;
    while (counter < OUTER_ITERS)
    {
        uint32_t t_start = HAL_GetTick();
            for (uint32_t i = 0; i < INNER_ITERS; i++)
            {
                float measurement = 1.0f + 0.1f * (float)(i & 0xF);
                float estimate = SimpleKalmanFilter_UpdateEstimate(&kf, measurement);
                sink = estimate;
            }

        counter++;
    }
        while (1)
        {
            __WFI();  // Wait For Interrupt → Sleep mode between interrupts
        }


    // FFT
//    FFT_InitTwiddles();
//    FFTBuffer fft_buf;
//
//    for (uint32_t n = 0; n < FFT_N; n++)
//    {
//        float t = (float)n / (float)FFT_N;
//        fft_buf.real[n] = sinf(2.0f * (float)M_PI * 5.0f * t);
//        fft_buf.imag[n] = 0.0f;
//    }
//
//    volatile float sink = 0.0f;
//
//    for (uint32_t iter = 0; iter < 10000; iter++)
//    {
//        FFT_Compute(&fft_buf);
//
//        float re = fft_buf.real[1];
//        float im = fft_buf.imag[1];
//        sink += re * re + im * im;
//    }
//
//    while (1)
//    {
//        __WFI();
//    }

    // EEG Filter
//        FIR_Init();
//
//        volatile float sink = 0.0f;
//
//        for (uint32_t iter = 0; iter < 3000; iter++)
//        {
//            FIR_Apply(eeg_in, eeg_out, EEG_SAMPLES);
//
//            float acc = 0.0f;
//            for (uint32_t n = 0; n < EEG_SAMPLES; n++)
//            {
//                acc += eeg_out[n] * eeg_out[n];
//            }
//            sink += acc;
//        }
//
//        while (1)
//        {
//            __WFI();
//        }
    return 0;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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
void SystemClock_Config_HighPerf(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // 1. High voltage scale
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    // 2. Enable HSI + LSI + PLL for 170 MHz
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    // 3. Select PLL as system clock
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

void SystemClock_Config_LowPower(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // 1. Use HSI directly (no PLL), still at current VOS (probably 1)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;  // turn PLL off
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    // 2. Switch SYSCLK to HSI (16 MHz), reduce Flash latency
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider  = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();

    // 3. Now that SYSCLK is low, we can safely lower the voltage
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
    // Optionally wait until VOS ready flag is cleared/ set, depending on family
    // while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {}
}

void SwitchToHighPerf(void)
{
    // 1. Raise voltage first
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    // wait until VOS ready if needed
    // while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {}

    // 2. Reconfigure clock to 170 MHz
    SystemClock_Config_HighPerf();
}

void SwitchToLowPower(void)
{
    // 1. Drop clock first (still at VOS1)
    SystemClock_Config_LowPower();

    // 2. Lower voltage to VOS2 (already done inside in this example; or do it here)
//    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/* USER CODE BEGIN 4 */

/**
  * @brief  System Clock Speed decrease
  *         The system Clock source is shifted from PLL to HSI
  * @param  None
  * @retval None
  */
void SystemClock_Decrease(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Select HSI as system clock source a */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Modify HSI to HSI DIV8 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief SYSTICK callback
  * @param None
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
  HAL_IncTick();
}

/**
  * @brief  Wake Up Timer callback
  * @param  hrtc : hrtc handle
  * @retval None
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{ 
  /* NOTE : add the specific code to handle the RTC wake up interrupt */
  /* Initialize LED2 on the board */
  BSP_LED_Init(LED2);
  /* On The LED2 */
  BSP_LED_On(LED2);

}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_PIN)
  {  
    UserButtonStatus = 1;
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
  /* Infinite loop */
  while (1)
  {
   /* User can add his own implementation to report the HAL error return state */
   /* Turn on the LED2 */
   BSP_LED_On(LED2);
   HAL_Delay(1000);
   BSP_LED_Off(LED2);
   HAL_Delay(200); 
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
 /* Infinite loop */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
