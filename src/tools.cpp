#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::endl;
using std::cout;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  /**
   * TODO: Calculate the RMSE here. DONE
   */
  cout << "RMSE" << endl;
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() != ground_truth.size())
  {
    cout << "Error estimations and truth are not the same size " << endl;
    return rmse;
  }

  for (int i = 0; i < estimations.size(); ++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  rmse = rmse / estimations.size();
  // rmse = rmse.array().sqrt();
  rmse = sqrt(rmse.array());
  cout << "RMSE DONE" << endl;
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{
  /**
   * TODO:
   * Calculate a Jacobian here. DONE
   */
  cout << "CalculateJacobian" << endl;
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px * px + py * py;

  //Avoid division by 0
    if(c1 < 0.00001)
    {
        px += 0.001;
        py += 0.001;
        c1 = px * px + py * py;
    }
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  // // check division by zero
  // if (fabs(c1) < 0.0001)
  // {
  //   std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
  //   return Hj;
  // }

  // compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;
cout << "CalculateJacobian Done" << endl;
  return Hj;
}
