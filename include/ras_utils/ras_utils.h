#ifndef RAS_UTILS_H
#define RAS_UTILS_H
#include <ctime>


namespace RAS_Utils
{
int sign(double a);
double time_diff_ms(const std::clock_t &begin, const std::clock_t &end);

}

#endif // RAS_UTILS_H
