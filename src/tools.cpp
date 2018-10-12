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
	VectorXd RMSE = VectorXd(4);
	RMSE << 0,0,0,0;
	
	// check vector sizes
	if (estimations.size() == 0) {
		std::cout << "Estimations vector is empty..." << std::endl;
		return RMSE;
	}

	if (estimations.size() != ground_truth.size()) {
		std::cout << "Estimations and ground truth vectors must be same size..." << std::endl;
		return RMSE;
	}

	// sum square difference
	for (unsigned int i=0; i < estimations.size(); ++i) {
		VectorXd diff = ground_truth[i] - estimations[i];		
		VectorXd sqr_diff = diff.array().square();
		RMSE += sqr_diff;
	}
	
	// mean
	RMSE /= estimations.size();

	// square root
	RMSE = RMSE.array().sqrt();

	return RMSE;
}
