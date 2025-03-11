#pragma once
#include "main.h"

#define TrajectoryPointsCount 100

# define EnableCarrier 0

# define TwinTrap 0

// Circle Mode
#define EnableCircleMode 1

#if EnableCircleMode
# define CircleFrequency 1U

# define CirclePeriodInMillisecond (1.0/CircleFrequency)*1000U

# define CircleRadius 0.01
#endif

#define TestFocusPointDistance 0.01;


#pragma pack(push, 1)
// Simulation Class
typedef struct 
{
  // Simulation Parameters
  float position[3]; // X Axis is Along Row, Y Axis Along Column, and Z Axis Target Outside Direction of the Array. In Meters, Mean Values.
  float strength;     // Overall strength Coefficient, Default to 100
  float spread; // Variance of the distribution for the target position
}Point;

Point Trajectory[TrajectoryPointsCount] = {0};

void CreateCircleTrajectory();

float EulerDistance(const float[3], const float[3]);
