#define _USE_MATH_DEFINES
#include "calibration.h"
#include "transducer.h"
#include "dma_manager.h"
#include "utiles.h"

int calibration_mode = 1;

// 校准参数说明：每个元素对应换能器的延迟校准值（单位：微秒us）
float Transducer_Calibration_Array[] = {
    26, 10, 24, 23, 22.5, 11, 21, 12,
    11.5, 7.5, 25, 10, 23, 10, 9.5, 22,
    4, 25, 12.5, 23.5, 8, 6.5, 20, 24,
    10, 24, 10, 6, 24, 22.5, 22, 8,
    23.5, 24, 24.5, 22, 19.5, 9.5, 10.5, 8,
    6, 7, 8, 10, 22, 22.5, 24, 20.5,
    9, 11, 6.5, 22, 11.5, 16, 10, 22,
    26.5, 4, 25, 8.5, 14, 8.5, 10, 0,
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
            Clean_Transducers_Calib();
        }
        else
        {
            Set_Transducers_Calib();
        }
        Update_All_DMABuffer();
    }
    lastKey0State = currentKey0State;
}

int Get_Calibration_Mode()
{
    return calibration_mode;
}


