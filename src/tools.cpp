#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() == 0 ||estimations.size() != ground_truth.size()) {
      cout << "Error" << endl;
      return rmse;
  }
  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
      rmse = (estimations[i] - ground_truth[i]).array()*
             (estimations[i] - ground_truth[i]).array() + rmse.array();
  }

  //calculate the mean
  rmse = rmse.array()/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  if (px == 0 && py == 0) {
      cout << "CalculateJacobian - Error - Division by Zero" << endl;

  //compute the Jacobian matrix
  } else {
      Hj << px/sqrt(pow(px, 2) + pow(py, 2)), py/sqrt(pow(px, 2) + pow(py, 2)), 0, 0,
            -py/(pow(px, 2) + pow(py, 2)), px/(pow(px, 2) + pow(py, 2)), 0, 0,
            py*(vx*py-vy*px)/pow((pow(px, 2) + pow(py, 2)),3/2), px*(vy*px-vx*py)/pow((pow(px, 2) + pow(py, 2)),3/2), px/sqrt(pow(px, 2) + pow(py, 2)), py/sqrt(pow(px, 2) + pow(py, 2));
  }
 
  return Hj;
}
