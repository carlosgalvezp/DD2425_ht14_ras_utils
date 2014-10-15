#include <ras_utils/controller.h>

Controller::Controller()
{
    kp_ = 0.0;
    ki_ = 0.0;
    kd_ = 0.0;
    prev_e_ = 0.0;
    sum_e_ = 0.0;
}

Controller::Controller(double kp, double kd, double ki)
{
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
    prev_e_ = 0.0;
    sum_e_ = 0.0;
}

void Controller::setData(double x_reference, double x_measured)
{
    x_ref_   = x_reference;
    x_measured_ = x_measured;
}

double Controller::computeControl()
{
    double e = x_ref_ - x_measured_;
    double u = kp_ * e + kd_ * (e - prev_e_) + ki_ * sum_e_;

    std::cout << "ERROR ["<<e<<";"<<kp_*e <<","<<kd_*prev_e_<<","<<ki_*sum_e_<<"]: " << e << std::endl;

    prev_e_ = e;
    sum_e_ = sum_e_ + e;
    std::cout << "U:  "<<u<<std::endl;
    return u;
}
