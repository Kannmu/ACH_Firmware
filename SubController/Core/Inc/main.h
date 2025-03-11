/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Transducer Class
typedef struct Transducer
{
  uint8_t Index;
  uint8_t row;
  uint8_t column;
  double coordinate[3];

  GPIO_TypeDef *port;
  uint8_t port_num;
  uint16_t pin;
  uint16_t calib;

  double distance;
  double phase;
  uint16_t shift_buffer_bits;

  float duty;
}Transducer;

// Simulation Class
typedef struct 
{
  // Simulation Parameters
  double position[3]; // X Axis is Along Row, Y Axis Along Column, and Z Axis Target Outside Direction of the Array. In Meters, Mean Values.
  float strength;     // Overall strength Coefficient, Default to 100
  double spread; // Variance of the distribution for the target position
}Simulation;

// extern struct Transducer TransducerArray[64];

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// extern uint8_t SYSTICK;
// extern uint16_t transducer_base_freq;

extern uint16_t LEDTicks;
extern uint32_t Timebase;
extern uint8_t DMA_Enable_Flag;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// void Error_Handler(void);
// void ToggleTransducer(Transducer T);
// void UpdateTransducers(Simulation S, Transducer T[]);

void CalculateFPS(void); 
void SendDebuggingInfo(void);
void EnableDMAs(void);
void DisableDMAs(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_5
#define KEY1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_10
#define LED0_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

// define the stage flags
# define EnableCarrier 0

# define PhaseTuningMode 0

# define CircleMode 1

# define TwinTrap 0

# define CircleFrequency 1U

# define CirclePeriodInMillisecond (1.0/CircleFrequency)*1000U

# define CircleRadius 0.01

# define LED0_GPIO_Port_Num 0U

# define pi 3.1415926535
# define SoundSpeed 343.2
# define NumTransducer (sizeof(transducer_pins) / sizeof(transducer_pins[0]))

# define transducer_base_freq 40000UL

# define Wave_K ((2*pi*transducer_base_freq)/SoundSpeed)

# define DMABaseNum (1024*6515) 

# if EnableCarrier

# define CarrierFrequency 200U
# define DMA_Buffer_Resolution (DMABaseNum/CarrierFrequency)
# define MainWaveLengthInBuffer (DMABaseNum/(transducer_base_freq))
# define DMA_Buffer_Update UpdateDMA_BufferWith_Duty_AM()

# else

# define DMA_Buffer_Resolution ((uint16_t)(DMABaseNum/transducer_base_freq))
# define MainWaveLengthInBuffer (DMA_Buffer_Resolution)
# define DMA_Buffer_Update UpdateDMA_Buffer()

# endif

# define TimeGapPerDMABufferBit ((long double)(1.0/(transducer_base_freq*DMA_Buffer_Resolution)))

# define BufferGapPerMicroseconds ((float)(1e-6)/TimeGapPerDMABufferBit)

# define TransducerPeriod  ((long double)(1.0 / transducer_base_freq))

# define WaveLength (TransducerPeriod*SoundSpeed)

# define ArraySize 8

# define TransducerGap (16.602 * (1e-3))

#define TOGGLE_PIN(port, pin) ((port)->ODR ^= (pin))
#define SET_PIN(port, pin, value) ((value)?((port)->BSRR = (pin)):((port)->BSRR = (pin) << 16))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
