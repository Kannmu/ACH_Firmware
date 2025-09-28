#pragma once
#include "main.h"
#include "transducer.h"

extern int calibration_mode; // 0 for not in calibration mode, 1 for in calibration mode

// calibration.h
extern float Transducer_Calibration_Array[];

void Calibration_Init(void);
void WaitNextKeyPressed(void);
void Switch_Calibration_Mode(void);
int Get_Calibration_Mode(void);
