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

  //-------------------------------------------------------------------------------
  //
  // code entered June-09-2017
  //
  //-------------------------------------------------------------------------------

  // initialize return vector
  VectorXd rmse(4);
           rmse << 0,0,0,0;

  // perform checks
  
  if (estimations.size() == 0 || (estimations.size() > ground_truth.size()) ) {
     
    cout << "ERROR (CalculateRMSE): vector size not matching or zero, returning zero" << endl;
    return rmse;

  }    

  // calculate

  // 1. residual
  for (int i=0; i < estimations.size(); ++i) {
	    
     VectorXd buf = estimations[i] - ground_truth[i];
              buf = buf.array() * buf.array();
              // update residual
  	      rmse += buf;
  }
  
  // 2. mean residual
  rmse = rmse / estimations.size(); 
  // 3. square root (wurzel)
  rmse = rmse.array().sqrt();

  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
 
  //-------------------------------------------------------------------------------
  //
  // code entered June-09-2017
  //
  //-------------------------------------------------------------------------------

  // matrix to return
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // perform divide by zero check
  const float ZERO_LIMIT = 0.001;  // ref. in lesson 0.01

  if (fabs(py) < ZERO_LIMIT && fabs(px) < ZERO_LIMIT) {
    
    cout << "ERROR (CalculateJacobian): divison by semi-zero = " << ZERO_LIMIT << endl;
    return Hj;
  }  

  // calculate matrix
  float pxy2sum   = px*px + py*py;
  float pxy2sum_s = sqrt(pxy2sum);
  float vxpy      = vx * py;
  float vypx      = vy * px; 
  float vpn       = pxy2sum * pxy2sum_s;
 
   // matrix elements
  float h11     = px / pxy2sum_s; 
  float h12     = py / pxy2sum_s;
  float h21     = -(py / pxy2sum);
  float h22     = px / pxy2sum;
  float h31     = py * (vxpy - vypx ) / vpn;
  float h32     = py * (vypx - vxpy ) / vpn;
  
  // matrix
  Hj <<   h11, h12, 0.0, 0.0,
          h21, h22, 0.0, 0.0,
          h31, h32, h11, h12;
          
  return Hj; 
  
}
