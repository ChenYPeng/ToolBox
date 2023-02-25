#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <vector>

#include "utils.h"

Eigen::Matrix4f calculateCameraToTool(const std::string& handeye_path);
Eigen::Matrix4f calculateToolTobase(const std::string& pose_path);
Eigen::Matrix4f calculateCameraToBase(const std::string& handeye_path, const std::string& pose_path);

Eigen::Matrix4f calculateCameraToTool(const std::string& handeye_path)
{
	std::ifstream inf;
	inf.open(handeye_path.c_str(), std::ios::in);

	if (!inf.is_open())
	{
		printf("Cannot Open File: s%", handeye_path.c_str());
		exit(-1);
	}

	std::vector<float> c2b;
	float x1;
	while (inf >> x1)
	{
		c2b.push_back(x1);
	}

	Eigen::Matrix4f T_camera2tool = Eigen::Map<Eigen::Matrix4f>(c2b.data()).transpose();

	return T_camera2tool;
}

Eigen::Matrix4f calculateToolTobase(const std::string& pose_path)
{
	// ��ȡ�����˱任����
	std::ifstream inf;
	inf.open(pose_path.c_str(), std::ios::in);
	if (!inf.is_open())
	{
		printf("Cannot Open File: s%", pose_path.c_str());
	}

	std::vector<float> ep;
	float x0;
	while (inf >> x0)
	{
		ep.push_back(x0);
	}

	// תΪ����
	Eigen::VectorXf euler_pose = Eigen::Map<Eigen::VectorXf>(ep.data(), ep.size());
	Eigen::AngleAxisf ra = Eigen::AngleAxisf(euler_pose[3] * PI / 180.0, Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf rb = Eigen::AngleAxisf(euler_pose[4] * PI / 180.0, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rc = Eigen::AngleAxisf(euler_pose[5] * PI / 180.0, Eigen::Vector3f::UnitX());

	// ��ת����
	Eigen::Matrix3f R;
	R = ra * rb * rc;// ZYX��ת ��� 

	// ƽ������
	Eigen::Vector3f T;
	T << euler_pose[0], euler_pose[1], euler_pose[2];

	// �任����
	Eigen::Matrix4f T_tool2base = Eigen::Matrix4f::Identity(); // 4x4 ��λ����
	T_tool2base.block<3, 3>(0, 0) = R; // ��ȡ���СΪ(p,q),��ʼ��(i,j)
	T_tool2base.block<3, 1>(0, 3) = T; // ��ȡ���СΪ(p,q),��ʼ��(i,j)

	return T_tool2base;
}


Eigen::Matrix4f calculateCameraToBase(const std::string& handeye_path, const std::string& pose_path)
{
	// ��ȡ���۾���
	Eigen::Matrix4f T_camera2tool = calculateCameraToTool(handeye_path.c_str());

	// ��ȡ������任
	Eigen::Matrix4f T_tool2base = calculateToolTobase(pose_path.c_str());

	// ����任����
	Eigen::Matrix4f T_camera2base = T_tool2base * T_camera2tool;

	return T_camera2base;
}



