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
    out_max_ = 1000;
    prev_e_ = 0.0;
    sum_e_ = 0.0;
}

Controller::Controller(double kp, double kd, double ki, double out_max)
{
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
    out_max_ = out_max;
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
    sum_e_ += ki_*e;

    if(fabs(sum_e_) > out_max_)
        sum_e_ = out_max_ * RAS_Utils::sign(sum_e_);

    double u = kp_ * e + kd_ * (e - prev_e_) + sum_e_;

    prev_e_ = e;
    return u;
}

int Controller::computeControl_Sat()
{
    return saturate(computeControl());
}

double Controller::saturate(const double x)
{
    if(x > out_max_)
        return out_max_;
    else if (x < -out_max_)
        return -out_max_;
    else
        return x;
}
