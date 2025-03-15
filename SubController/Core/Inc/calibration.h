#pragma once
#include "main.h"
#include "transducer.h"
# define PhaseTuningMode 0


// calibration.h
extern float Transducer_Calibration_Array[];

void StartCalibration(void);
void WaitNextKeyPressed(void);