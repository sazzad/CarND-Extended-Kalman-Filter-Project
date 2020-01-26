#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}




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
