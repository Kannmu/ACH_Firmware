/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : calibration.c
 * @brief          : calibration
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

#define _USE_MATH_DEFINES
#include "calibration.h"
#include "transducer.h"
#include "dma_manager.h"
#include "debug.h"

int calibration_mode = 0; // 0 for not in calibration mode, 1 for in calibration mode

// 校准参数说明：每个元素对应换能器的延迟校准值（单位：微秒us）
float Transducer_Calibration_Array[] ={
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0
};

void Switch_Calibration_Mode()
{
    static GPIO_PinState lastKey0State = GPIO_PIN_SET;
    GPIO_PinState currentKey0State = HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);
    // Toggle Calibration Mode
    if(lastKey0State == GPIO_PIN_SET && currentKey0State == GPIO_PIN_RESET)
    {
        calibration_mode = 1 - calibration_mode;
    }
    lastKey0State = currentKey0State;
    // Update LED1 State
    if(calibration_mode)
    {
        for (int i = 0; i < DMA_Buffer_Resolution; i++)
        {
            DMA_Buffer[0U][i] &= ~LED1_Pin; // 拉低引脚：通过清零对应位实现
        }
        Update_All_DMABuffer(Raw);
    }
    else
    {
        for (int i = 0; i < DMA_Buffer_Resolution; i++)
        {
            DMA_Buffer[0U][i] |= LED1_Pin; // 拉高引脚：通过设置对应位实现
        }
    }
}


int Get_Calibration_Mode()
{
    return calibration_mode;
}


