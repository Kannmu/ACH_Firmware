#include "simulation.h"

Point Trajectory[TrajectoryPointsCount] = {0};

void CreateCircleTrajectory()
{
    for (size_t i = 0l; i < TrajectoryPointsCount; i++)
    {
        Trajectory[i].position[0] = -CircleRadius * sin((2 * pi * (Timebase / CirclePeriodInMillisecond)));
        Trajectory[i].position[1] = CircleRadius * cos((2 * pi * (Timebase / CirclePeriodInMillisecond)));
        Trajectory[i].position[2] = TestFocusPointDistance;
    }
}

float EulerDistance(const float From[3], const float To[3])
{
    float TempDistance = 0.0;
    for (int i = 0; i < 3; i++)
    {
        float diff = From[i] - To[i];
        TempDistance += diff * diff; // Avoid calling pow() for exponent 2, use direct multiplication
    }
    return sqrt(TempDistance);
}
