#define _USE_MATH_DEFINES
#include "debug.h"
#include "simulation.h"
#include "calibration.h"

// debug.c
const uint16_t HALF_LED_BLINK_PERIOD = 500U;
uint16_t led0_ticks = 0;
uint32_t sysTickDelta= 0;
uint32_t FPS = 0;
int led0_state = 0;
int last_led0_state = 0;
uint16_t stm_test_ticks = 0;


void Calculate_FPS()
{
    static uint32_t lastTick = 0;
    uint32_t currentTick = HAL_GetTick();
    uint32_t tickDelta = currentTick - lastTick;
    
    if(tickDelta >= 1000) { // 每秒更新一次
        // 使用SysTick计数器计算更精确的FPS
        uint32_t cycleCount = (SystemCoreClock / 1000) * tickDelta;
        FPS = cycleCount / sysTickDelta;
        lastTick = currentTick;
    }
}

void LED_Indicate_Blink()
{
    uint8_t progress = led0_ticks / (HALF_LED_BLINK_PERIOD);
    if (progress < 1)
    {
        led0_state = 0;
    }
    else if (progress >= 1 && progress < 2)
    {
        led0_state = 1;
    }
    else if (progress >= 2)
    {
        led0_ticks = 0U;
    }
    
    if (led0_state != last_led0_state)
    {
        last_led0_state = led0_state;
        Set_LED_State(LED0_Pin, led0_state);
    }
}

void Send_Debugging_Info()
{


}

void HAL_Delay_us(uint32_t nus)
{
    uint32_t told, tnow, tcnt = 0;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;
            told = tnow;
            if (tcnt >= nus * (SystemCoreClock / 1000000))
                break;
        }
    };
}

void Restore_LED_State()
{
    for (int i = 0; i < DMA_Buffer_Resolution; i++)
    {
        // Restore Indicate LED State
        if (led0_state)
        {
            DMA_Buffer[0][i] |= LED0_Pin; // LED 亮
        }
        else
        {
            DMA_Buffer[0][i] &= ~LED0_Pin; // LED 灭
        }

        // Restore Calibration LED State
        if (Get_Calibration_Mode())
        {
            DMA_Buffer[0][i] |= LED1_Pin; // LED 亮
        }
        else
        {
            DMA_Buffer[0][i] &= ~LED1_Pin; // LED 灭
        }

        // Restore Simulation LED State
        if (Get_Simulation_Mode())
        {
            DMA_Buffer[0][i] |= LED2_Pin; // LED 亮
        }
        else
        {
            DMA_Buffer[0][i] &= ~LED2_Pin; // LED 灭
        }
    }
}

void Set_LED_State(uint16_t pin, int state)
{
    for (int i = 0; i < DMA_Buffer_Resolution; i++)
    {
        if (state)
        {
            DMA_Buffer[0][i] |= pin; // LED 亮
        }
        else
        {
            DMA_Buffer[0][i] &= ~pin; // LED 灭
        }
    }
}

void Toggle_LED_State(uint16_t pin)
{
    for (int i = 0; i < DMA_Buffer_Resolution; i++)
    {
        DMA_Buffer[0][i] ^= pin; // LED 切换状态
    }
}

