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

    double time_diff_ms(timeval *begin, timeval *end)
    {
        return 1000.0*(end->tv_sec  - begin->tv_sec) +
                0.001*(end->tv_usec - begin->tv_usec);
    }

    double time_diff_ms(const ros::WallTime &begin, const ros::WallTime &end)
    {
        return (end.toNSec() - begin.toNSec())/1000000.0;
    }


    double sensorToDistanceInCM(int sensor_val, std::vector<double> polynomial_coof)
    {
        double distance = 0;
        for(int n = 0; n < polynomial_coof.size(); n++) {
            distance += polynomial_coof[n] * pow(sensor_val, n);
        }
        return distance;
    }

    double shortSensorToDistanceInCM(int sensor_val)
    {
        return sensorToDistanceInCM(sensor_val, {
                                    262.503338214074,
                                    -4.88675316092840,
                                    0.0415096360540543,
                                    -0.000191007269073580,
                                    4.90926172893023e-07,
                                    -6.62146190833533e-10,
                                    3.64763123318473e-13
                                    });
    }

    double longSensorToDistanceInCM(int sensor_val)
    {
        return sensorToDistanceInCM(sensor_val, {
                                    2.53149410424676e-13,
                                    -5.23382644623806e-10,
                                    4.37076393642977e-07,
                                    -0.000188802004989696,
                                    0.0447961028587077,
                                    -5.68757150017330,
                                    329.045444431424
                                    });
    }
}
