#define _USE_MATH_DEFINES
#include "debug.h"

// debug.c
const uint8_t LIVE_LED_PERIOD = 1;
uint16_t led_ticks = 0;
uint32_t sysTickDelta= 0;
uint32_t FPS = 0;
uint32_t time_base = 0;

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
    const uint16_t LED_BLINK_THRESHOLD = LIVE_LED_PERIOD * 500U;
    
    if (led_ticks >= LED_BLINK_THRESHOLD)
    {
        // 使用DMA缓冲区控制LED
        for (int i = 0; i < DMA_Buffer_Resolution; i++)
        {
            DMA_Buffer[LED0_GPIO_Port_Num][i] ^= LED0_Pin;
        }
        led_ticks = 0U;
    }
    led_ticks++;  // 添加计数器递增
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
