#ifndef RAS_SENSOR_UTILS_H
#define RAS_SENSOR_UTILS_H

namespace RAS_Utils
{

namespace sensors
{




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
    if(sensor_val > 440) {
        return 4.5;
    }


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
                                329.045444431424,
                                -5.68757150017330,
                                0.0447961028587077,
                                -0.000188802004989696,
                                4.37076393642977e-07,
                                -5.23382644623806e-10,
                                2.53149410424676e-13,
                                });
}

struct SensorDistances
{
    double back_;
    double front_;
    double right_front_;
    double right_back_;
    double left_front_;
    double left_back_;
    SensorDistances(){}
    SensorDistances(int front_reading, int back_reading, int right_front_reding, int right_back_reading, int left_front_reading, int left_back_reading)
        : front_(longSensorToDistanceInCM(front_reading)),
          back_(longSensorToDistanceInCM(back_reading)),
          right_front_(shortSensorToDistanceInCM(right_front_reding)),
          right_back_(shortSensorToDistanceInCM(right_back_reading)),
          left_front_(shortSensorToDistanceInCM(left_front_reading)),
          left_back_(shortSensorToDistanceInCM(left_back_reading))
    {
    }
};

}

}

#endif // RAS_SENSOR_UTILS_H
