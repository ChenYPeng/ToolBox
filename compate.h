#pragma once
#include <random>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "utils.h"

Eigen::Matrix3d skew(Eigen::Vector3d u);

Eigen::MatrixXd SvdInverse(Eigen::MatrixXd A);

void solove_SVD(const Eigen::MatrixXf H, Eigen::MatrixXf &U, Eigen::MatrixXf &V);

Eigen::MatrixXd Solve_Axxb(const std::vector<Eigen::MatrixXd> AA, const std::vector<Eigen::MatrixXd> BB);



double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
	return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
}

double getTranslationError(Eigen::Vector3d t_exp, Eigen::Vector3d t_est) {
	return (t_exp - t_est).norm();
}

Eigen::Matrix3d matrixExp3(const Eigen::Vector3d& w, float theta) {
	Eigen::Matrix3d bw, R;
	bw << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
	R << Eigen::Matrix3d::Identity() + std::sin(theta)*bw + (1 - std::cos(theta))*bw*bw;
	return R;
}

Eigen::Matrix4d getT(const Eigen::Matrix3d R, const Eigen::Vector3d& t) {
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	T.topLeftCorner(3, 3) = R;
	T.topRightCorner(3, 1) = t;
	return T;
}
Eigen::Matrix4d getT(const Eigen::Vector3d& w, float theta, const Eigen::Vector3d& t) {
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	T.topLeftCorner(3, 3) = matrixExp3(w.normalized(), theta);
	T.topRightCorner(3, 1) = t;
	return T;
}

Eigen::Matrix4d getRandomT() {
	// random seeds
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis_num(-1, 1);
	std::uniform_real_distribution<> dis_angle(0, 2 * PI);

	// uniformly generate unit vector (as rotation axis) by equal-area projection
	Eigen::Vector3d w;
	float z = dis_num(gen);
	float alpha = dis_angle(gen);
	w << std::sqrt(1 - z * z)*std::cos(alpha), std::sqrt(1 - z * z)*std::sin(alpha), z;

	// generate translation in [-1, 1] cube
	Eigen::Vector3d t;
	t << dis_num(gen), dis_num(gen), dis_num(gen);

	return getT(w, dis_angle(gen), t);
}
Eigen::Matrix3d skew(Eigen::Vector3d u)
{
	Eigen::Matrix3d u_hat = Eigen::MatrixXd::Zero(3, 3);
	u_hat(0, 1) = u(2);
	u_hat(1, 0) = -u(2);
	u_hat(0, 2) = -u(1);
	u_hat(2, 0) = u(1);
	u_hat(1, 2) = u(0);
	u_hat(2, 1) = -u(0);

	return u_hat;
}

Eigen::MatrixXd SvdInverse(Eigen::MatrixXd A)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
	double  pinvtoler = 1.e-6; //tolerance
	int row = A.rows();
	int col = A.cols();
	int k = std::min(row, col);
	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
	Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
	Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
	for (long i = 0; i < k; ++i) {
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else singularValues_inv(i) = 0;
	}
	for (long i = 0; i < k; ++i)
	{
		singularValues_inv_mat(i, i) = singularValues_inv(i);
	}
	X = (svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());//X=VS+U*

	return X;
}

void solove_SVD(const Eigen::MatrixXf H, Eigen::MatrixXf &U, Eigen::MatrixXf &V)
{
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	U = svd.matrixU(); // left_singular_vectors
	V = svd.matrixV(); // right_singular_vectors
}

Eigen::MatrixXd Solve_Axxb(const std::vector<Eigen::MatrixXd> AA, const std::vector<Eigen::MatrixXd> BB) {

	assert(AA.size() == BB.size());
	Eigen::Matrix4d Hx = Eigen::Matrix4d::Identity(); //待求解标定矩阵
	int num = AA.size();//观察数据组个数
	std::cout << "数据对个数:" << num << std::endl;
	//MatrixXd output=MatrixXd::Identity();
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num * 3, 3); // solve Ax=B for rotation
	Eigen::VectorXd B = Eigen::MatrixXd::Zero(num * 3, 1);
	//solve rotation
	for (size_t i = 0; i < AA.size(); i++) {
		//将旋转矩阵转化为旋转轴形式
		Eigen::AngleAxisd rgij, rcij;
		rgij.fromRotationMatrix(AA[i].block<3, 3>(0, 0));
		rcij.fromRotationMatrix(BB[i].block<3, 3>(0, 0));
		double theta_gij = rgij.angle();
		double theta_cij = rcij.angle();
		Eigen::Vector3d rngij = rgij.axis();
		Eigen::Vector3d rncij = rcij.axis();
		Eigen::AngleAxisd Pgij(2 * sin(theta_gij / 2), rngij);
		Eigen::AngleAxisd Pcij(2 * sin(theta_cij / 2), rncij);

		Eigen::Vector3d pgij_vector = Pgij.axis()*Pgij.angle();
		Eigen::Vector3d pcij_vector = Pcij.axis()*Pcij.angle();
		Eigen::Vector3d vec_sum_axis = pgij_vector + pcij_vector;
		Eigen::Matrix3d S = skew(vec_sum_axis);
		Eigen::Vector3d vec_sub_axis = pcij_vector - pgij_vector;


		//cout << S << endl;
		//AngleAxisd sub_axis(Pcij.matrix()*Pgij.matrix().inverse());

		//Vector3d vec_sub_axis =sub_axis.axis()*sub_axis.angle();

		//A(3 * i - 2 : 3 * i, 1 : 3) = S;
		//B.block<>(3 * i - 2 : 3 * i, 1) = vec_sub_axis;
		A.block<3, 3>(3 * i, 0) = S;
		//cout << A.block<3, 3>(3 * i, 0) << endl;
		B.block<3, 1>(3 * i, 0) = vec_sub_axis;
	}
	//求解旋转
	Eigen::VectorXd Pce1 = SvdInverse(A)*B;
	Eigen::VectorXd Pce = 2 * Pce1 / sqrt(1 + Pce1.norm()*Pce1.norm());
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	double np2 = Pce.norm()*Pce.norm();
	Eigen::MatrixXd Rx = (1 - np2 * 0.5)*I + 0.5*(Pce*Pce.transpose() + sqrt(4 - np2)*skew(Pce));
	std::cout << "旋转矩阵是否正交:" << Rx * Rx.transpose() << std::endl;
	//求解平移
	A.setZero();
	B.setZero();
	for (size_t i = 0; i < AA.size(); i++) {
		Eigen::Vector3d T_A, T_B;
		T_A = AA[i].block<3, 1>(0, 3);
		T_B = BB[i].block<3, 1>(0, 3);
		Eigen::MatrixXd R_A = AA[i].block<3, 3>(0, 0);
		A.block<3, 3>(3 * i, 0) = R_A - I;
		B.block<3, 1>(3 * i, 0) = Rx * T_B - T_A;
	}
	Eigen::VectorXd Tx = SvdInverse(A)*B;
	Hx.block<3, 3>(0, 0) = Rx;
	Hx.block<3, 1>(0, 3) = Tx;
	return Hx;
}

Eigen::Vector3f GaussNewton(Eigen::Matrix2Xf& data, Eigen::Vector3f& guess) {
	// model: f = exp(a*x*x + b*x + c);
	int size = 100;
	Eigen::Vector3f var = guess;
	for (int t = 0; t < 10; ++t) {
		float a = var(0);
		float b = var(1);
		float c = var(2);
		Eigen::Matrix3Xf Jacobian(3, size);
		Eigen::VectorXf Residual(size);
		for (int i = 0; i < size; ++i) {
			float x = data(0, i);
			Residual(i) = data(1, i) - std::exp(a*x*x + b * x + c);
			Jacobian(0, i) = x * x * std::exp(a*x*x + b * x + c);
			Jacobian(1, i) = x * std::exp(a*x*x + b * x + c);
			Jacobian(2, i) = std::exp(a*x*x + b * x + c);
		}
		var += (Jacobian * Jacobian.transpose()).inverse() * Jacobian * Residual;
		std::cout << "current var = " << var << std::endl;
	}
	return var;
}

//求解协方差矩阵
Eigen::MatrixXd& cov(Eigen::MatrixXd& outMatrix, const Eigen::MatrixXd& inMatrix)
{
	Eigen::MatrixXd meanVec = inMatrix.colwise().mean();
	Eigen::RowVectorXd meanVecRow(Eigen::RowVectorXd::Map(meanVec.data(), inMatrix.cols()));
	Eigen::MatrixXd zeroMeanMat = inMatrix;
	zeroMeanMat.rowwise() -= meanVecRow;
	if (inMatrix.rows() == 1)
	{
		outMatrix = (zeroMeanMat.adjoint() * zeroMeanMat) / double(inMatrix.rows());
	}
	else
	{
		outMatrix = (zeroMeanMat.adjoint() * zeroMeanMat) / double(inMatrix.rows() - 1);
	}
	return outMatrix;
}

//伪逆矩阵(Moore-Penrose pseudoinverse)A定义：
//A+=VD+UT,其中，U，D和V是矩阵A奇异值分解后得到的矩阵。对角矩阵D的伪逆D+是非零元素取倒数之后再转置得到的。
Eigen::MatrixXd& pinv(Eigen::MatrixXd& outMatrix, const Eigen::MatrixXd& inMatrix)
{
	double pinvtoler = 1.e-6; // choose your tolerance wisely!
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(inMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXd singularValues_inv = svd.singularValues();
	Eigen::VectorXd sv = svd.singularValues();
	for (Eigen::Index i = 0; i < svd.cols(); ++i)
	{
		if (sv(i) > pinvtoler)
		{
			singularValues_inv(i) = 1.0 / sv(i);
		}
		else
		{
			singularValues_inv(i) = 0;
		}
	}
	outMatrix = (svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose());
	return outMatrix;
}