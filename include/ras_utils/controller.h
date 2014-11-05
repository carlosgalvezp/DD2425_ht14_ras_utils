#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <iostream>
#include <cmath>
#include <ras_utils/ras_utils.h>

class Controller
{
public:

    /**
     * @brief Default constructor
     */
    Controller();

    /**
     * @brief PID controller
     * @param kp P gain
     * @param kd D gain
     * @param ki I gain
     */
    Controller(double kp, double kd, double ki);

    /**
     * @brief PID controller
     * @param kp P gain
     * @param kd D gain
     * @param ki I gain
     */
    Controller(double kp, double kd, double ki, double out_max);

    /**
     * @brief Set reference and measured value
     * @param x_reference
     * @param x_measured
     */
    void setData(double x_reference, double x_measured);

    /**
     * @brief Compute control signal
     * @return u
     */
    double computeControl();

    int computeControl_Sat();

private:
    double kp_, kd_, ki_;
    double prev_e_, sum_e_;
    double x_ref_, x_measured_;
    double out_max_;

    double saturate(const double x);

};

#endif // CONTROLLER_H
