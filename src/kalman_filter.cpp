#include <ras_utils/kalman_filter.h>
using namespace Eigen;

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::KalmanFilter(const Eigen::VectorXd& mu0,
                           const Eigen::MatrixXd& sigma0,
                           const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& B,
                           const Eigen::MatrixXd& C,
                           const Eigen::MatrixXd& R,
                           const Eigen::MatrixXd& Q)
{
    // ** Check matrices dimensions
    int n = mu0.rows();     // Size of state vector

    assert(sigma0.rows() == sigma0.cols());
    assert(sigma0.rows() == n);

    assert(A.rows() == A.cols());
    assert(A.rows() == n);

    assert(B.rows() == n);

    assert(R.rows() == R.cols());
    assert(R.rows() == n);

    // ** Initialize variables
    A_ = A;
    B_ = B;
    C_ = C;
    R_ = R;
    Q_ = Q;
    sigma_ = sigma0;
    mu_ = mu0;

    I_ = MatrixXd::Identity(mu0.rows(), mu0.rows());
}

void KalmanFilter::filter(const Eigen::VectorXd& u,
                          const Eigen::VectorXd& z,
                          Eigen::VectorXd& x_new,
                          Eigen::MatrixXd& sigma_new)
{
    // ** Predict   
    Eigen::VectorXd mu_bar    = A_*mu_ + B_*u;
    Eigen::MatrixXd sigma_bar = A_*sigma_*A_.transpose() + R_;

    // ** Update
    Eigen::MatrixXd S = C_*sigma_bar*C_.transpose() + Q_;
    Eigen::MatrixXd K = sigma_bar*C_.transpose()* S.inverse();

    mu_    = mu_bar + K*(z - C_*mu_bar);
    sigma_ = (I_ - K*C_)*sigma_bar;

    // ** Output
    x_new = mu_;
    sigma_new = sigma_;
}

void KalmanFilter::filter(const Eigen::VectorXd &z, Eigen::VectorXd &x_new)
{
    Eigen::VectorXd v_dummy(B_.cols());
    Eigen::MatrixXd m_dummy(sigma_.rows(), sigma_.cols());

    this->filter(v_dummy, z, x_new, m_dummy);
}
