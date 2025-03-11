#include "debug.h"

// debug.c
const uint8_t LiveLEDPeriod = 1;
uint16_t LEDTicks = 0;
uint16_t DeltaTicks = 0;
uint32_t FPS = 0;
uint32_t Timebase = 0;


void CalculateFPS()
{
    // Calculate DeltaTicks
    if (SysTick->VAL < DeltaTicks)
    {
        DeltaTicks = SysTick->LOAD - DeltaTicks + SysTick->VAL;
    }
    else
    {
        DeltaTicks = SysTick->VAL - DeltaTicks;
    }

    // Calculate FPS
    FPS = (SystemCoreClock / (float)DeltaTicks);
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
    char TargetStr[100];
    const char str[4] = "FPS:";

    strcat(TargetStr, str);

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
