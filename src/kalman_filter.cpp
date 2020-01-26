#include "kalman_filter.h"
#include "tools.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */


void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);

}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}


void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;

#if 0
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
#else
  MatrixXd K_transpose = S.ldlt().solve(H_*P_);
  MatrixXd K = K_transpose.transpose();
#endif

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}



void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  Tools tools;

  VectorXd z_pred = tools.GetHx(x_);
  VectorXd y = z - z_pred;

  y(1) = tools.NormalizeAngle(y(1));



  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
#if 0
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
#else
  MatrixXd K_transpose = S.ldlt().solve(H_*P_);
  MatrixXd K = K_transpose.transpose();
#endif

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}
