#ifndef RAS_UTILS_H
#define RAS_UTILS_H

#include <ctime>
#include <sys/time.h>
#include <vector>
#include <math.h>
#include "ros/ros.h"

namespace RAS_Utils
{

int sign(double a);
double time_diff_ms(const std::clock_t &begin, const std::clock_t &end);
double time_diff_ms(struct timeval *begin, struct timeval *end);
double time_diff_ms(const ros::WallTime &begin, const ros::WallTime &end);

double shortSensorToDistanceInCM(int sensor_val);
double sensorToDistanceInCM(int sensor_val, std::vector<double> polynomial_coof);
}

#endif // RAS_UTILS_H
