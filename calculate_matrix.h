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
	// 读取机器人变换矩阵
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

	// 转为向量
	Eigen::VectorXf euler_pose = Eigen::Map<Eigen::VectorXf>(ep.data(), ep.size());
	Eigen::AngleAxisf ra = Eigen::AngleAxisf(euler_pose[3] * PI / 180.0, Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf rb = Eigen::AngleAxisf(euler_pose[4] * PI / 180.0, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rc = Eigen::AngleAxisf(euler_pose[5] * PI / 180.0, Eigen::Vector3f::UnitX());

	// 旋转矩阵
	Eigen::Matrix3f R;
	R = ra * rb * rc;// ZYX旋转 左乘 

	// 平移向量
	Eigen::Vector3f T;
	T << euler_pose[0], euler_pose[1], euler_pose[2];

	// 变换矩阵
	Eigen::Matrix4f T_tool2base = Eigen::Matrix4f::Identity(); // 4x4 单位矩阵
	T_tool2base.block<3, 3>(0, 0) = R; // 提取块大小为(p,q),起始于(i,j)
	T_tool2base.block<3, 1>(0, 3) = T; // 提取块大小为(p,q),起始于(i,j)

	return T_tool2base;
}


Eigen::Matrix4f calculateCameraToBase(const std::string& handeye_path, const std::string& pose_path)
{
	// 读取手眼矩阵
	Eigen::Matrix4f T_camera2tool = calculateCameraToTool(handeye_path.c_str());

	// 读取基坐标变换
	Eigen::Matrix4f T_tool2base = calculateToolTobase(pose_path.c_str());

	// 计算变换矩阵
	Eigen::Matrix4f T_camera2base = T_tool2base * T_camera2tool;

	return T_camera2base;
}



