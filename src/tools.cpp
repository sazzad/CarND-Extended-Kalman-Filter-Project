#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector4d;
using std::vector;



VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
  if (px==0 and py==0){
      std::cout<<"Error: division by zero\n";
      return Hj;
  }
  double a = px*px + py*py;
  double asqr = sqrt(a);
  Hj(0,0) = px/asqr;
  Hj(0,1) = py/asqr;
  Hj(0,2) = 0.;
  Hj(0,3) = 0.;
  
  Hj(1,0) = -py/a;
  Hj(1,1) = px/a;
  Hj(1,2) = 0.;
  Hj(1,3) = 0.;
  
  Hj(2,0) = py*(py*vx - px*vy)/(a*asqr);
  Hj(2,1) = px*(px*vy - py*vx)/(a*asqr);
  Hj(2,2) = px/asqr;
  Hj(2,3) = py/asqr;
  
  return Hj;
}

VectorXd Tools::GetHx(const VectorXd& x){
  double px = x(0);
  double py = x(1);
  double vx = x(2);
  double vy = x(3);

  double rho = sqrt(px*px+py*py);
  double theta = atan2(py, px);
  double rhodot = (px*vx+py*vy)/sqrt(px*px+py*py);

  VectorXd hx = VectorXd(3);
  hx<< rho, theta, rhodot;

  return hx;
}

MatrixXd Tools::CalculateJacobianNumerical(const VectorXd& x) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  double h = 0.0001;
  VectorXd y1 = (GetHx(x + Vector4d(h, 0, 0, 0))  - GetHx(x - Vector4d(h, 0, 0, 0)))/(2*h);
  VectorXd y2 = (GetHx(x + Vector4d(0, h, 0, 0))  - GetHx(x - Vector4d(0, h, 0, 0)))/(2*h);
  VectorXd y3 = (GetHx(x + Vector4d(0, 0, h, 0))  - GetHx(x - Vector4d(0, 0, h, 0)))/(2*h);
  VectorXd y4 = (GetHx(x + Vector4d(0, 0, 0, h))  - GetHx(x - Vector4d(0, 0, 0, h)))/(2*h);

  Hj.block<3,1>(0,0) = y1;
  Hj.block<3,1>(0,1) = y2;
  Hj.block<3,1>(0,2) = y3;
  Hj.block<3,1>(0,3) = y4;


  return Hj;
}

double Tools::NormalizeAngle(double theta){
  while ( theta > M_PI || theta < -M_PI ) {
    if ( theta > M_PI ) {
      theta -= 2*M_PI;
    } else {
      theta += 2*M_PI;
    }
  }
  return theta;
}
