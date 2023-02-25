#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>

#include "utils.h"

void quaternionToRotationMatrix(const Quaternion q, Eigen::Matrix3d &R);
void rotationMatrixToQuaternion(Eigen::Matrix3d R, const Quaternion &q);

void eulerAngleToQuaternion(const EulerAngle angle, Quaternion &q);
void quaternionToEulerAngle(const Quaternion q, EulerAngle &angle);

bool isRotationMatrix(const Eigen::Matrix3f R);
void eulerAngleToRotationMatrix(const EulerAngle angle, Eigen::Matrix3f &rotR, std::string &poseType);
void eulerAngleToTransformMatrix(const ToolPose pose, Eigen::Matrix4f & T, std::string &poseType);

void rotationMatrixToEulerAngle(const Eigen::Matrix3f R, EulerAngle &angle, std::string &poseType);
void transformMatrixToeulerAngle(const Eigen::Matrix4f T, Eigen::VectorXf &eulerAngle, std::string &poseType);


void quaternionToRotationMatrix(const Quaternion q, Eigen::Matrix3d &R)
{
	// Q  = qw + qxi +qyj +qzk
	double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
	double q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
	double q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
	double q1q2 = q1 * q2, q1q3 = q1 * q3;
	double q2q3 = q2 * q3;

	R << (q0q0 + q1q1 - q2q2 - q3q3), 2 * (q1q2 + q0q3), 2 * (q1q3 - q0q2),
		2 * (q1q2 - q0q3), (q0q0 - q1q1 + q2q2 - q3q3), 2 * (q2q3 + q0q1),
		2 * (q1q3 + q0q2), 2 * (q2q3 - q0q1), (q0q0 - q1q1 - q2q2 + q3q3);
}

void rotationMatrixToQuaternion(Eigen::Matrix3d R, const Quaternion &q)
{

}

void eulerAngleToQuaternion(const EulerAngle angle, Quaternion &q)
{
	double cy = cos(angle.rx * 0.5);
	double sy = sin(angle.rx * 0.5);
	double cp = cos(angle.ry * 0.5);
	double sp = sin(angle.ry * 0.5);
	double cr = cos(angle.rz * 0.5);
	double sr = sin(angle.rz * 0.5);

	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;
}

void quaternionToEulerAngle(const Quaternion q, EulerAngle &angle)
{
	// rx (x-axis rotation)
	double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	angle.rx = std::atan2(sinr_cosp, cosr_cosp);

	// ry (y-axis rotation)
	double sinp = 2 * (q.w * q.y - q.z * q.x);
	if (std::abs(sinp) >= 1)
		angle.ry = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		angle.ry = std::asin(sinp);

	// rz (z-axis rotation)
	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	angle.rz = std::atan2(siny_cosp, cosy_cosp);
}

bool isRotationMatrix(const Eigen::Matrix3f R)
{
	double err = 1e-6;
	Eigen::Matrix3f Rt = R.transpose();
	Eigen::Matrix3f shouldBeIdentity = Rt * R;
	Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

	return  (shouldBeIdentity - I).norm() < err;
}

// 已验证
void eulerAngleToRotationMatrix(const EulerAngle angle, Eigen::Matrix3f &rotR, std::string &poseType)
{
	double rx = angle.rx / 180.0 * PI;
	double ry = angle.ry / 180.0 * PI;
	double rz = angle.rz / 180.0 * PI;

	Eigen::Matrix3f rotX, rotY, rotZ;

	rotX << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
	rotY << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
	rotZ << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;

	if (poseType == "ZYX")
	{
		rotR = rotX * rotY * rotZ;// ZYX旋转 左乘 
	}
	else if (poseType == "YZX")
	{
		rotR = rotX * rotZ * rotY;// YZX旋转 左乘
	}
	else if (poseType == "ZXY")
	{
		rotR = rotY * rotX * rotZ;// ZXY旋转 左乘 
	}
	else if (poseType == "YXZ")
	{
		rotR = rotZ * rotX * rotY;// YXZ旋转 左乘 
	}
	else if (poseType == "XYZ")
	{
		rotR = rotZ * rotY * rotX;// XYZ旋转 左乘 
	}
	else if (poseType == "XZY")
	{
		rotR = rotY * rotZ * rotX;// XZY旋转 左乘  
	}
	else
	{
		std::cout << "Euler Angle Sequence string is wrong..." << std::endl;
	}

	if (!isRotationMatrix(rotR))
	{
		std::cout << "Euler Angle convert to RotatedMatrix failed..." << std::endl;
		exit(-1);
	}
}

void eulerAngleToTransformMatrix(const ToolPose pose, Eigen::Matrix4f & T, std::string &poseType)
{

	Eigen::AngleAxisf rotX = Eigen::AngleAxisf(pose.rx * PI / 180.0, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf rotY = Eigen::AngleAxisf(pose.ry * PI / 180.0, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rotZ = Eigen::AngleAxisf(pose.rz * PI / 180.0, Eigen::Vector3f::UnitZ());

	// 旋转矩阵
	Eigen::Matrix3f rotR;

	if (poseType == "ZYX")
	{
		rotR = rotX * rotY * rotZ;// ZYX旋转 左乘 
	}
	else if (poseType == "YZX")
	{
		rotR = rotX * rotZ * rotY;// YZX旋转 左乘
	}
	else if (poseType == "ZXY")
	{
		rotR = rotY * rotX * rotZ;// ZXY旋转 左乘 
	}
	else if (poseType == "YXZ")
	{
		rotR = rotZ * rotX * rotY;// YXZ旋转 左乘 
	}
	else if (poseType == "XYZ")
	{
		rotR = rotZ * rotY * rotX;// XYZ旋转 左乘 
	}
	else if (poseType == "XZY")
	{
		rotR = rotY * rotZ * rotX;// XZY旋转 左乘  
	}
	else
	{
		std::cout << "Euler Angle Sequence string is wrong..." << std::endl;
	}

	if (!isRotationMatrix(rotR))
	{
		std::cout << "Euler Angle convert to RotatedMatrix failed..." << std::endl;
		exit(-1);
	}

	// 平移向量
	Eigen::Vector3f tranT(pose.x, pose.y, pose.z);

	// 变换矩阵
	T = Eigen::Matrix4f::Identity(); // 4x4 单位矩阵
	T.block<3, 3>(0, 0) = rotR; // 提取块大小为(p,q),起始于(i,j)
	T.block<3, 1>(0, 3) = tranT; // 提取块大小为(p,q),起始于(i,j)
}

// ZYX
void rotationMatrixToEulerAngle(const Eigen::Matrix3f R, EulerAngle &angle, std::string &poseType)
{
	assert(isRotationMatrix(R));

	double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

	if (poseType == "ZYX")
	{
		if (!sy < 1e-6)
		{
			angle.rx = atan2(R(2, 1), R(2, 2)) / PI * 180.0;
			angle.ry = atan2(-R(2, 0), sy) / PI * 180.0;
			angle.rz = atan2(R(1, 0), R(0, 0)) / PI * 180.0;
		}
		else
		{
			angle.rx = atan2(-R(1, 2), R(1, 1)) / PI * 180.0;
			angle.ry = atan2(-R(2, 0), sy) / PI * 180.0;
			angle.rz = 0;
		}
	}
}

// ZYX
void transformMatrixToeulerAngle(const Eigen::Matrix4f T, Eigen::VectorXf &eulerAngle, std::string &poseType)
{
	float x, y, z, rx, ry, rz;
	eulerAngle = Eigen::VectorXf(6);

	Eigen::Matrix3f R = T.block<3, 3>(0, 0);

	assert(isRotationMatrix(R));

	Eigen::Vector3f t = T.block<3, 1>(0, 3);

	x = t(0), y = t(1), z = t(2);

	float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

	if (!sy < 1e-6)
	{
		rx = atan2(R(2, 1), R(2, 2)) / PI * 180.0;
		ry = atan2(-R(2, 0), sy) / PI * 180.0;
		rz = atan2(R(1, 0), R(0, 0)) / PI * 180.0;
	}
	else
	{
		rx = atan2(-R(1, 2), R(1, 1)) / PI * 180.0;
		ry = atan2(-R(2, 0), sy) / PI * 180.0;
		rz = 0;
	}

}
