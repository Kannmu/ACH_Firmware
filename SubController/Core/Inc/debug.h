#pragma once
#include "main.h"
#include "dma_manager.h"

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
# define LED0_GPIO_Port_Num 0U

// Debug Parameters
// debug.h
extern const uint8_t LiveLEDPeriod;
extern uint16_t LEDTicks;
extern uint16_t DeltaTicks;
extern uint32_t FPS;
extern uint32_t Timebase;


void IndicateLEDBlink();
void SendDebuggingInfo();
void CalculateFPS();
void HAL_Delay_us(uint32_t nus);
