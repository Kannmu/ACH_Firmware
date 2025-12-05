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
extern const uint8_t LIVE_LED_PERIOD;
extern uint16_t led0_ticks;
extern uint32_t sysTickDelta;
extern uint32_t FPS;
extern uint16_t stm_test_ticks;
extern float System_Loop_Freq;
extern double updateDMABufferDeltaTime;

void LED_Indicate_Blink();
void Calculate_FPS();
void HAL_Delay_us(uint32_t nus);
void Restore_LED_State();
void Set_LED_State(uint16_t pin, int state);
void Toggle_LED_State(uint16_t pin);
float Get_Voltage(void);
float Get_Temperature(void);


