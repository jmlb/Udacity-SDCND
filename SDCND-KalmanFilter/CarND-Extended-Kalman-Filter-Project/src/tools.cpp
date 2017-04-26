#include <iostream>
#include "tools.h"

//define a threshold value
#define epsilon 0.0001
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here
  if (estimations.size() == ground_truth.size() && estimations.size() !=0)
  {
      
      for(int i=0; i < estimations.size(); ++i){
          //accumulate squared residuals
          VectorXd residual = estimations[i] - ground_truth[i];
          residual = residual.array() * residual.array();
          rmse += residual;
      }
      //calculate the mean
      // ... your code here
      rmse = rmse / estimations.size();
      //calculate the squared root
      // ... your code here
      rmse = rmse.array().sqrt();
  }
  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //TODO: YOUR CODE HERE 

  //check division by zero
  
  if (px*px + py*py == 0)
  {
    return Hj;
  }
  else
  {
    float norm_p = px*px + py*py;
    Hj(0, 0) = px / sqrt(norm_p);
    Hj(1, 0) = -py / (norm_p);
    Hj(2, 0) = py *(vx * py - vy*px) / pow(norm_p, 3/2);
    Hj(0, 1) = py / sqrt(norm_p);
    Hj(1, 1) = px / (norm_p);
    Hj(2, 1) = px *(vy * px - vx*py) / pow(norm_p, 3/2);
    Hj(0, 2) = 0;
    Hj(1, 2) = 0;
    Hj(2, 2) = px / sqrt(norm_p);
    Hj(0, 3) = 0;
    Hj(1, 3) = 0;
    Hj(2, 3) = py / sqrt(norm_p);
    return Hj;
  }
}

