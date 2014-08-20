// Author: Enrico Corvaglia
// pseudo_inverse() computes the pseudo inverse of Jacobian J_
// returns the pseudo inverted Jacobian J_pinv_

#ifndef PSEUDO_INVERSION_H
#define PSEUDO_INVERSION_H

#include <kdl/kdl.hpp>
#include <Eigen/Core>
#include <Eigen/LU>

inline void pseudo_inverse(KDL::Jacobian &J_, Eigen::Matrix<double,7,6> &J_pinv_)
{
	Eigen::Matrix<double,6,6> temp_ = Eigen::Matrix<double,6,6>::Zero();
	J_pinv_ = Eigen::Matrix<double,7,6>::Zero();

	// computing J*J^T
	for (int i = 0; i < J_.rows(); i++)	// 6
		for (int j = 0; j < J_.rows(); j++)	// 6
			for (int k = 0; k < J_.columns(); k++)	// 7
				temp_(i,j) += J_(i,k)*J_(j,k);

	// computing (J*J^T)^-1
	temp_ = temp_.inverse();

	// computing J^T(J*J^T)^-1
	for (int i = 0; i < J_.columns(); i++)	// 7
		for (int j = 0; j < J_.rows(); j++)	// 6
			for (int k = 0; k < J_.rows(); k++)	// 6
				J_pinv_(i,j) += J_(k,i)*temp_(k,j);

}
#endif