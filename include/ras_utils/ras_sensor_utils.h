#ifndef RAS_SENSOR_UTILS_H
#define RAS_SENSOR_UTILS_H



namespace RAS_Utils
{
namespace sensors
{

#define MAX_DIST_SIDE_WALL 25

#define SHORT_SENSOR_DISTANCE_FROM_CENTER           10.68
#define SHORT_RIGHT_FRONT_SENSOR_ANGLE_FROM_FORWARD -0.53839


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

bool canFollowWall(double d_front, double d_back, double max_dist = MAX_DIST_SIDE_WALL)
{
    return d_front < max_dist && d_back < max_dist;
}

bool canFollowLeftWall(const RAS_Utils::sensors::SensorDistances &sd)
{
    return canFollowWall(sd.left_front_, sd.left_back_);
}

bool canFollowRightWall(const RAS_Utils::sensors::SensorDistances &sd)
{
    return canFollowWall(sd.right_front_, sd.right_back_);
}

bool canFollowWall(const RAS_Utils::sensors::SensorDistances &sd, bool right_wall)
{
    if(right_wall)
    {
        return canFollowRightWall(sd);
    }else
    {
        return canFollowLeftWall(sd);
    }
}

bool canFollowAWall(const RAS_Utils::sensors::SensorDistances &sd)
{
    return canFollowLeftWall(sd) || canFollowRightWall(sd);
}

double getDistanceToLeftWall(const RAS_Utils::sensors::SensorDistances &sd)
{
    return  0.5*(sd.left_back_ + sd.left_front_);
}

double getDistanceToRightWall(const RAS_Utils::sensors::SensorDistances &sd)
{
    return 0.5*(sd.right_back_ + sd.right_front_);
}

double getDistanceToClosestWall(const RAS_Utils::sensors::SensorDistances &sd)
{
    return fmin(getDistanceToLeftWall(sd), getDistanceToRightWall(sd));
}

bool shouldPrioritizeRightWall(const RAS_Utils::sensors::SensorDistances &sd)
{
    if(!canFollowRightWall(sd)) return false;
    if(!canFollowLeftWall(sd)) return true;
    return getDistanceToRightWall(sd) < getDistanceToLeftWall(sd);
}

double getShortSensorAngle(bool right_side) {
    return (right_side) ? (3*M_PI/2) : M_PI / 2.0;
}

double getShortSensorAngleCenterOffset(bool right_side, bool front)
{
    double angle = (front) ? SHORT_RIGHT_FRONT_SENSOR_ANGLE_FROM_FORWARD : M_PI - SHORT_RIGHT_FRONT_SENSOR_ANGLE_FROM_FORWARD;
    angle *= (right_side) ? 1 : -1;
    return angle;
}



}}

#endif // RAS_SENSOR_UTILS_H
