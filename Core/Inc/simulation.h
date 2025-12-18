# pragma once
# include "main.h"
# include "transducer.h"

typedef struct Point
{
  // Simulation Parameters
  float position[3]; // X Axis is Along Row, Y Axis Along Column, and Z Axis Target Outside Direction of the Array. In Meters, Mean Values.
  float strength;     // Overall strength Coefficient, Default to 100
  float vibration[3]; // Vibration distance Coefficient in XYZ Axis, Default to 0 for No Vibration
  float frequency;
  int phase; // 0 for original position, 1 for potions + vibration.
}Point;

extern Point FocusPoint;

void Switch_Simulation_Mode(void);
int Get_Simulation_Mode(void);
void Update_Focus_Point(Point *point);
void Apply_Vibration(void);
