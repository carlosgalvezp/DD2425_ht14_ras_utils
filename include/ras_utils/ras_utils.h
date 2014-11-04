#include <vector>

#ifndef RAS_UTILS_H
#define RAS_UTILS_H

namespace RAS_Utils
{
    int sign(double a);

    double shortSensorToDistanceInCM(int sensor_val);

    double sensorToDistanceInCM(int sensor_val, std::vector<double> polynomial_coof);
}

#endif // RAS_UTILS_H
