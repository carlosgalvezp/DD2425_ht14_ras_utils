#include <ras_utils/ras_utils.h>

namespace RAS_Utils
{

    int sign(double a)
    {
        if (a >= 0) return 1;
        else        return -1;
    }

    double time_diff_ms(const std::clock_t &begin, const std::clock_t &end)
    {
        return 1000.0*(double)(end-begin)/(double)CLOCKS_PER_SEC;
    }
}
