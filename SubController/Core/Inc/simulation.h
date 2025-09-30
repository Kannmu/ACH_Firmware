# pragma once
# include "main.h"
# include "transducer.h"

typedef struct Point
{
  // Simulation Parameters
  float position[3]; // X Axis is Along Row, Y Axis Along Column, and Z Axis Target Outside Direction of the Array. In Meters, Mean Values.
  float strength;     // Overall strength Coefficient, Default to 100
  float spread; // Variance of the distribution for the target position
}Point;

// simulation.h

void Switch_Simulation_Mode(void);
int Get_Simulation_Mode(void);

