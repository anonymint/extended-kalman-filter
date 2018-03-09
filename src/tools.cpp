#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	VectorXd rsme(4);
	rsme << 0,0,0,0;

	if ( estimations.size() == 0 || estimations.size() != ground_truth.size() ) {
		std::cout << "estimations or ground_truth vector size problem either 0 or not the same size" << std::endl;
		return rsme;
	}

	for (unsigned int i = 0; i < estimations.size(); i++) {
		VectorXd diff = estimations[i] - ground_truth[i];
		diff = diff.array()*diff.array();
		rsme += diff; 
	}

	rsme = rsme / estimations.size();

	rsme = rsme.array().sqrt();

	return rsme;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  	
  	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}
