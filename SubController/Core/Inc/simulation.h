#pragma once
#include "main.h"
#include "transducer.h"

# define TrajectoryPointsCount 100

# define TrajectoryRefreshRate 100U

# define TrajectoryRefreshGap (1.0/TrajectoryRefreshRate)

#define TestFocusPointDistance 0.01;

typedef struct Point
{
  // Simulation Parameters
  float position[3]; // X Axis is Along Row, Y Axis Along Column, and Z Axis Target Outside Direction of the Array. In Meters, Mean Values.
  float strength;     // Overall strength Coefficient, Default to 100
  float spread; // Variance of the distribution for the target position
}Point;

// simulation.h
extern Point *Trajectory[TrajectoryPointsCount];

void Create_Test_Trajectory();

float Euler_Distance(const float[3], const float[3]);
