#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <sys/time.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/LU>

class KalmanFilter
{
public:
    KalmanFilter();
    KalmanFilter(const Eigen::VectorXd& mu0,
                 const Eigen::MatrixXd& sigma0,
                 const Eigen::MatrixXd& A,
                 const Eigen::MatrixXd& B,
                 const Eigen::MatrixXd& C,
                 const Eigen::MatrixXd& R,
                 const Eigen::MatrixXd& Q
                 );
    void filter(const Eigen::VectorXd &u, const Eigen::VectorXd& z, Eigen::VectorXd& x_new, Eigen::MatrixXd &sigma_new);

private:
    Eigen::VectorXd mu_;     // state mean
    Eigen::MatrixXd sigma_;  // state covariance
    Eigen::MatrixXd A_;      // A matrix (state dynamics)
    Eigen::MatrixXd B_;      // B matrix (control)
    Eigen::MatrixXd C_;      // C matrix (observability)
    Eigen::MatrixXd R_;      // Process error
    Eigen::MatrixXd Q_;      // Measurement error

    Eigen::MatrixXd I_;      // Identity matrix
};

#endif // KALMAN_FILTER_H
