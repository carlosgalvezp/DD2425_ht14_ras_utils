#ifndef RAS_UTILS_H
#define RAS_UTILS_H

#include <ctime>
#include <sys/time.h>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include <ras_srv_msgs/Command.h>
#include <ras_utils/ras_names.h>

#define PRINT_PADDING 5

namespace RAS_Utils
{

int sign(double a);
double time_diff_ms(const std::clock_t &begin, const std::clock_t &end);
double time_diff_ms(struct timeval *begin, struct timeval *end);
double time_diff_ms(const ros::WallTime &begin, const ros::WallTime &end);
double time_diff_ns(const ros::WallTime &begin, const ros::WallTime &end);

double shortSensorToDistanceInCM(int sensor_val);
double longSensorToDistanceInCM(int sensor_val);
double sensorToDistanceInCM(int sensor_val, std::vector<double> polynomial_coof);

void print(const std::string & text);
void print(const std::string & text, const double value, std::string & padding);
void print(const std::string & text, const double value);
void print(const std::string & text, const double value1, const double value2);
void print(const std::vector<std::string> & texts, const std::vector<double> & values);

double mean(const std::vector<double> &data);
double std(const std::vector<double> &data, double mu);
double std(const std::vector<double> &data);

double mahalanobis_distance(const double &x, const double &mu, const double &sigma);

}

#endif // RAS_UTILS_H
