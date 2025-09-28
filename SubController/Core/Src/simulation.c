/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : simulation.c
 * @brief          : simulation
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
#include "simulation.h"

Point *Trajectory[TrajectoryPointsCount] = {0};

void Create_Test_Trajectory()
{

}


float Euler_Distance(const float From[3], const float To[3])
{
    float TempDistance = 0.0;
    for (int i = 0; i < 3; i++)
    {
        float diff = From[i] - To[i];
        TempDistance += diff * diff; // Avoid calling pow() for exponent 2, use direct multiplication
    }
    return sqrt(TempDistance);
}
