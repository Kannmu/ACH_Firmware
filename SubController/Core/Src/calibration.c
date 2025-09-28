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
float Transducer_Calibration_Array[] = {
    24, 10, 24, 23, 23, 11, 21, 12,
    11.5, 7, 23.5, 9.5, 23, 8, 9, 22,
    5, 24, 12, 22, 8, 10, 20, 24,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
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

        if(calibration_mode)
        {
            Update_All_DMABuffer(Raw);
        }
        else
        {
            Update_All_DMABuffer(Calib);
        }
        // Update LED1 State
        if(calibration_mode)
        {
            for (int i = 0; i < DMA_Buffer_Resolution; i++)
            {
                DMA_Buffer[0U][i] &= ~LED1_Pin; // 拉低引脚：通过清零对应位实现
            }
        }
        else
        {
            for (int i = 0; i < DMA_Buffer_Resolution; i++)
            {
                DMA_Buffer[0U][i] |= LED1_Pin; // 拉高引脚：通过设置对应位实现
            }
        }
    }
    lastKey0State = currentKey0State;
}

int Get_Calibration_Mode()
{
    return calibration_mode;
}


