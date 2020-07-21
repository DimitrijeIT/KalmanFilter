#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
   x_ = F_ * x_ ;
   P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  MatrixXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px * px + py * py);

  //Avoid division by zero
  if (rho < 0.00001)
  {
    px += 0.001;
    py += 0.001;
    rho = sqrt(px * px + py * py);
  }

  float teta = atan2(py,px);
  float rho_dot = (px*vx+py*vy)/rho;

  VectorXd hx(3);
  hx << rho, teta, rho_dot;

  VectorXd y = z - hx;

  //Normalize angle to within -pi and pi if needed
  while (y(1) > M_PI || y(1) < -M_PI)
  {
    if (y(1) > M_PI)
    {
      y(1) -= 2 * M_PI;
    }
    else if (y(1) < -M_PI)
    {
      y(1) += 2 * M_PI;
    }
  }

  MatrixXd Ht = H_.transpose();
  
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  //New estimate
  x_ = x_ + (K * y);

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}
