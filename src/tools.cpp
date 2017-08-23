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

  // initialize vectors
  VectorXd rmse(4), diff(4), diff_sq(4);
  rmse.fill(0.0);

  // check estimation and ground-truth size
  if ((estimations.size() == 0) || (estimations.size() != ground_truth.size())) {
    cout << "ERROR: invalid estimations vector or ground_truth vector" << endl;

    if (estimations.size() == 0)
      cout << "ERROR-Type: estimations vector size = 0" << endl;
    else
      cout << "ERROR-Type: estimations-vector & ground_truth-vector size mismatch" << endl;

    return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i) {
      diff = estimations[i] - ground_truth[i];
      diff_sq = diff.array()*diff.array();
      rmse = rmse + diff_sq;
  }
  //calculate the mean
  rmse = rmse / estimations.size();
  //calculate the squared root
  rmse = rmse.array().sqrt();

  //std::cout << "RMSE calculated correctly" << std::endl;

  return rmse;
}
