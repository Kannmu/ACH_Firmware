#pragma once
#include "main.h"
#include "transducer.h"
# define EnableDutyAM 0

# define EnableOOK 0

# define TwinTrap 0

#define EnableCircleMode 1

# define TrajectoryPointsCount 100

# define TrajectoryRefreshRate 100U

# define TrajectoryRefreshGap (1.0/TrajectoryRefreshRate)

// Circle Mode

#if EnableCircleMode
# define CircleFrequency 1U

# define CirclePeriodInMillisecond (1.0/CircleFrequency)*1000U

# define CircleRadius 0.01
#endif

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

void CreateTestTrajectory();
void CreateCircleTrajectory();

float EulerDistance(const float[3], const float[3]);
