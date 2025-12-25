#define _USE_MATH_DEFINES
#include "simulation.h"
#include "calibration.h"
#include "utiles.h"

int simulation_mode = 1;
static uint32_t lastToggleTick = 0;

Point FocusPoint = {
    .position = {0.0f, 0.0f, 0.05f},
    .strength = 100,
    .vibration = {0.0f, 0.0f, 0.0f},
    .frequency = 0.0f,
    .phase = 0
};

Point IdlePoint = {
    .position = {0.0f, 0.0f, 0.05f},
    .strength = 0,
    .vibration = {0.0f, 0.0f, 0.0f},
    .frequency = 0.0f,
    .phase = 0
};


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
            Set_Focus_Point(FocusPoint.position);
        }
        Update_All_DMABuffer();
    }
    lastKey1State = currentKey1State;
}

int Get_Simulation_Mode()
{
    return simulation_mode;
}

void Update_Focus_Point(Point *point)
{
    FocusPoint = *point;
}

void Apply_Vibration()
{
    if (FocusPoint.frequency > 1.0f)
    {
        uint32_t interval = (uint32_t)(500.0f / FocusPoint.frequency);
        if (HAL_GetTick() - lastToggleTick >= interval)
        {
            FocusPoint.phase = 1 - FocusPoint.phase;
            lastToggleTick = HAL_GetTick();
        }
        else
        {
            return;
        }
    }
    else
    {
        return;
    }

    if(Get_Calibration_Mode() == 1 && Get_Simulation_Mode() == 1)
    {
        float pos[3];
        if (FocusPoint.phase == 0) {
            pos[0] = FocusPoint.position[0];
            pos[1] = FocusPoint.position[1];
            pos[2] = FocusPoint.position[2];
        } else {
            pos[0] = FocusPoint.position[0] + FocusPoint.vibration[0];
            pos[1] = FocusPoint.position[1] + FocusPoint.vibration[1];
            pos[2] = FocusPoint.position[2] + FocusPoint.vibration[2];
        }
        Set_Focus_Point(pos);
        Update_All_DMABuffer();
    }
}



