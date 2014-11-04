#include <ras_utils/ras_utils.h>
#include <vector>
#include <math.h>

namespace RAS_Utils
{

    int sign(double a)
    {
        if (a >= 0) return 1;
        else        return -1;
    }

    double sensorToDistanceInCM(int sensor_val, std::vector<double> polynomial_coof)
    {
        double distance = 0;
        for(int n = 0; n < polynomial_coof.size(); n++) {
            distance += polynomial_coof[n] * pow(sensor_val, n);
        }
        return distance;
    }

    double shortSensorToDistanceInCM(int sensor_val) {
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
}
