#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <iostream>

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


private:
    double kp_, kd_, ki_;
    double prev_e_, sum_e_;
    double x_ref_, x_measured_;
};

#endif // CONTROLLER_H
