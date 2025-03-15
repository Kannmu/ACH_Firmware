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

// 校准参数说明：每个元素对应换能器的延迟校准值（单位：微秒us）
float Transducer_Calibration_Array[] =
{
    8.8, 9.5, 11, 20.6, 7.4, 21.5, 21.5, 8.5,
    20.6, 21, 19.4, 7, 20, 20.6, 6.4, 20.6,
    19.4, 6.8, 19.5, 20.4, 7.3, 20, 8.2, 20.3,
    7.5, 19, 19.7, 6.5, 19.4, 7, 9.4, 9.5,
    9, 9.6, 5, 6.8, 18.3, 17.8, 19.3, 8.2,
    8.2, 20.4, 7.9, 6.7, 5.4, 7, 20.1, 21.6,
    19, 20.6, 19.2, 20.7, 7, 18.5, 6.8, 19.7,
    5.6, 10.6, 9.2, 20.3, 19.7, 6.4, 18.2, 2.2,
    0};

void StartCalibration()
{
    for(size_t i=0; i< NumTransducer;i++)
    {
        UpdateSingleDMABuffer(TransducerArray[i], Raw);
        WaitNextKeyPressed();
    }
}

void WaitNextKeyPressed()
{
    //Chose KEY 1 as Next KEY
    while(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)
    {
        // 等待按键被按下（假设按键按下时为低电平）
        HAL_Delay(10); // 添加短暂延时防止CPU占用过高
        IndicateLEDBlink(); // 在等待过程中保持LED闪烁
    }
    
    // 等待按键释放
    while(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(10);
    }
    
    // 按键消抖
    HAL_Delay(50);
}


