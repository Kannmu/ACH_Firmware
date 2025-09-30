# define _USE_MATH_DEFINES
# include "custom_math.h"
# include <math.h>

float Euler_Distance(const float From[3], const float To[3])
{
    float TempDistance = 0.0;
    for (int i = 0; i < 3; i++)
    {
        float diff = From[i] - To[i];
        TempDistance += diff * diff; // Avoid calling pow() for exponent 2, use direct multiplication
    }
    return (float)sqrt(TempDistance);
}

