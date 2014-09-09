// Author: Enrico Corvaglia
// pseudo_inverse() computes the pseudo inverse of matrix M_ using SVD decomposition (can choose between damped and not)
// returns the pseudo inverted matrix M_pinv_

#ifndef PSEUDO_INVERSION_H
#define PSEUDO_INVERSION_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
using namespace Eigen;

inline void pseudo_inverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_,bool damped = true)
{	
	double lambda_ = damped?0.2:0.0;

	JacobiSVD<MatrixXd> svd(M_, ComputeFullU | ComputeFullV);
	JacobiSVD<MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
	MatrixXd S_ = M_;	// copying the dimensions of M_, its content is not needed.
	S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

    M_pinv_ = MatrixXd(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
}

/*inline void pseudo_inverse(const Eigen::Matrix<double,6,7> &M_, Eigen::MatrixXd &M_pinv_)
{
	Eigen::Matrix<double,6,6> temp_ = Eigen::Matrix<double,6,6>::Zero();
	M_pinv_ = Eigen::Matrix<double,7,6>::Zero();

	// computing J*J^T
	for (int i = 0; i < M_.rows(); i++)	// 6
		for (int j = 0; j < M_.rows(); j++)	// 6
			for (int k = 0; k < M_.cols(); k++)	// 7
				temp_(i,j) += M_(i,k)*M_(j,k);

	// computing (J*J^T)^-1
	temp_ = temp_.inverse();

	// computing J^T(J*J^T)^-1
	for (int i = 0; i < M_.cols(); i++)	// 7
		for (int j = 0; j < M_.rows(); j++)	// 6
			for (int k = 0; k < M_.rows(); k++)	// 6
				M_pinv_(i,j) += M_(k,i)*temp_(k,j);

}*/

#endif
