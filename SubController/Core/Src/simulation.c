#define _USE_MATH_DEFINES
#include "simulation.h"
#include "calibration.h"
#include "utiles.h"

Point StaticFocusPoint = {
    .position = {0.0f, 0.0f, 0.0525f},
    .strength = 100,
    .vibration = {0.0f, 0.0f, 0.0f},
    .amplitude = 0.0f,
    .frequency = 0.0f
};

int simulation_mode = 0;

void Switch_Simulation_Mode()
{
    static GPIO_PinState lastKey1State = GPIO_PIN_SET;
    GPIO_PinState currentKey1State = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
    // Toggle Simulation
    if (lastKey1State == GPIO_PIN_SET && currentKey1State == GPIO_PIN_RESET)
    {
        simulation_mode = 1 - simulation_mode;
        if (simulation_mode == 0)
        {
            Set_Plane_Wave();
        }
        else if (simulation_mode == 1)
        {
            // Static Focus Point
            Set_Focus_Point(&StaticFocusPoint);
        }
        Update_All_DMABuffer();
        
    }
    lastKey1State = currentKey1State;
}

int Get_Simulation_Mode()
{
    return simulation_mode;
}

