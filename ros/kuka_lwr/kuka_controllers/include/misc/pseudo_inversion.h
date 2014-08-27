// Author: Enrico Corvaglia
// pseudo_inverse() computes the pseudo inverse of Jacobian J_ using SVD decomposition
// returns the pseudo inverted Jacobian J_pinv_

#ifndef PSEUDO_INVERSION_MULTI_H
#define PSEUDO_INVERSION_MULTI_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
using namespace Eigen;

inline void pseudo_inverse_DLS(const Eigen::Matrix<double,6,7> &J_, Eigen::Matrix<double,7,6> &J_pinv_)
{	
	double lambda_ = 0.1;

	JacobiSVD<MatrixXd> svd(J_, ComputeFullU | ComputeFullV);
	MatrixXd U_ = svd.matrixU();
	MatrixXd V_ = svd.matrixV();
	JacobiSVD<MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
	Matrix<double,6,7> S_ = Matrix<double,6,7>::Zero();

    for (int i = 0; i < S_.rows(); i++)
        S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

    J_pinv_ = V_*S_.transpose()*U_.transpose();

	//std::cout<<"U:"<<std::endl<<U_<<std::endl<<"V:"<<std::endl<<V_<<std::endl<<"S:"<<std::endl<<S_<<std::endl;
}

inline void pseudo_inverse(const Eigen::Matrix<double,6,7> &J_, Eigen::Matrix<double,7,6> &J_pinv_)
{
	Eigen::Matrix<double,6,6> temp_ = Eigen::Matrix<double,6,6>::Zero();
	J_pinv_ = Eigen::Matrix<double,7,6>::Zero();

	// computing J*J^T
	for (int i = 0; i < J_.rows(); i++)	// 6
		for (int j = 0; j < J_.rows(); j++)	// 6
			for (int k = 0; k < J_.cols(); k++)	// 7
				temp_(i,j) += J_(i,k)*J_(j,k);

	// computing (J*J^T)^-1
	temp_ = temp_.inverse();

	// computing J^T(J*J^T)^-1
	for (int i = 0; i < J_.cols(); i++)	// 7
		for (int j = 0; j < J_.rows(); j++)	// 6
			for (int k = 0; k < J_.rows(); k++)	// 6
				J_pinv_(i,j) += J_(k,i)*temp_(k,j);

}

#endif