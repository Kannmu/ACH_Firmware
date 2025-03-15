#define _USE_MATH_DEFINES
#include "debug.h"

// debug.c
const uint8_t LiveLEDPeriod = 1;
uint16_t LEDTicks = 0;
uint16_t sysTickDelta= 0;
uint32_t FPS = 0;
uint32_t Timebase = 0;

void CalculateFPS()
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

void IndicateLEDBlink()
{
    if (LEDTicks >= LiveLEDPeriod * 500)
    {
        // HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        for (int i = 0; i < DMA_Buffer_Resolution; i++)
        {
            DMA_Buffer[LED0_GPIO_Port_Num][i] ^= LED0_Pin;
        }
        LEDTicks = 0;
    }
}

void SendDebuggingInfo()
{
    // char TargetStr[100] = {0}; // 初始化为0
    // char fpsStr[20];
    
    // sprintf(fpsStr, "FPS:%lu", FPS);
    // strcat(TargetStr, fpsStr);
    
    // 如果需要发送数据
    // uint32_t len = strlen(TargetStr);
    // CDC_Transmit_FS(TargetStr, len);
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
