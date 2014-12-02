#ifndef RAS_UTILS_H
#define RAS_UTILS_H

#include <ctime>
#include <sys/time.h>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include <ras_srv_msgs/Command.h>
#include <ras_utils/ras_names.h>
#include <algorithm>

#define PRINT_PADDING 5

namespace RAS_Utils
{

int sign(double a);
double time_diff_ms(const std::clock_t &begin, const std::clock_t &end);
double time_diff_ms(struct timeval *begin, struct timeval *end);
double time_diff_ms(const ros::WallTime &begin, const ros::WallTime &end);
double time_diff_ns(const ros::WallTime &begin, const ros::WallTime &end);

void print(const std::string & text);
void print(const std::string & text, const double value, std::string & padding);
void print(const std::string & text, const double value);
void print(const std::string & text, const double value1, const double value2);
void print(const std::vector<std::string> & texts, const std::vector<double> & values);

double mean(const std::vector<double> &data);
double std(const std::vector<double> &data, double mu);
double std(const std::vector<double> &data);

double mahalanobis_distance(const double &x, const double &mu, const double &sigma);

double normalize_angle( double angle );

void normalize_probabilities(std::vector<double> &prob);

template<typename T>
T get_most_repeated(const typename std::vector<T> &vector)
{
    typename std::vector<T>::const_iterator it;
    typename std::vector<T> visited;
    typename std::vector<int> count;
    // ** Count the number of unique elements
    for(it = vector.begin(); it < vector.end(); ++it)
    {
        const T& obj = *it;
        typename std::vector<T>::iterator found = std::find(visited.begin(), visited.end(), obj);
        if(found == visited.end()) // Not yet seen
        {
            visited.push_back(obj);
            count.push_back(1);
        }
        else
        {
            ++count[found-visited.begin()];
        }
    }

    // ** Get maximum element
    int maxCount=0;
    T maxResult;
    for(std::size_t i=0; i < count.size(); ++i)
    {
        if(count[i] > maxCount)
        {
            maxCount = count[i];
            maxResult = visited[i];
        }
    }
    return maxResult;
}


template<typename Iterator>
double L2(Iterator begin, Iterator end)
{
    double result = 0.0;
    for(Iterator it =  begin; it < end; ++it)
    {
        result += (*it) * (*it);
    }
    return sqrt(result);
}

template<typename Iterator>
double dotProduct(Iterator begin1, Iterator end1, Iterator begin2, Iterator end2)
{
    double result = 0.0;
    for(Iterator it1 =  begin1, it2 = begin2; it1 < end1, it2 < end2; ++it1, ++it2)
    {
        result += (*it1) * (*it2);
    }
    return result;
}

}

#endif // RAS_UTILS_H
