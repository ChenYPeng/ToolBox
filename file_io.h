#pragma once
#include <fstream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "utils.h"

void loadPointCloud(const std::string& filename, pcl::PointCloud<PointT>::Ptr& cloud);
void loadPointCloudFromTxt(const std::string &file_path, const char tag, const pcl::PointCloud<pcl::PointXYZ>::Ptr &read_cloud);
void loadEulerAngleFormTxt(const std::string &file_path, std::vector<Eigen::VectorXf> &angles);
void loadMatrixPointFormTxt(const std::string &file_path, Eigen::MatrixXf &cloud_matrix);
void savePointCloudToFile(const pcl::PointCloud<PointT>::Ptr write_cloud, const std::string file_path);
void loadMatrix4fFormTxt(const std::string &file_path, Eigen::Matrix4f &matrix);
void saveMatrixToFile(const Eigen::Matrix4f matrix, const std::string &file_path);

void loadPointCloud(const std::string& filename, pcl::PointCloud<PointT>::Ptr &cloud)
{
	std::string extension = filename.substr(filename.size() - 4, 4);
	if (extension == ".pcd") {
		if (pcl::io::loadPCDFile<PointT>(filename, *cloud) == -1) {
			std::cout << "Was not able to open file " << filename << std::endl;
		}
	}
	else if (extension == ".ply") {
		if (pcl::io::loadPLYFile<PointT>(filename, *cloud) == -1) {
			std::cout << "Was not able to open file " << filename << std::endl;
		}
	}
	else {
		std::cerr << "Was not able to open file " << filename
			<< " (it is neither .pcd nor .ply) " << std::endl;
	}
}

/*-*-*-*-*-*-*-*-*-*-*-*- Txt => Cloud -*-*-*-*-*-*-*-*-*-*-*-*/
void loadPointCloudFromTxt(const std::string &file_path, const char tag, const pcl::PointCloud<pcl::PointXYZ>::Ptr &read_cloud)
{
	std::ifstream fin(file_path.c_str());
	if (!fin.is_open())
	{
		printf("Cannot Open File: %s", file_path.c_str());
	}

	std::string lineStr;
	while (getline(fin, lineStr)) 
	{
		if (lineStr.size() == 0) break;
		std::vector<std::string> strVector;
		std::string s;
		std::stringstream ss(lineStr);
		while (getline(ss, s, tag)) // tag 为 txt 文件分割符（',' 或 ' '）
		{
			strVector.push_back(s);
		}
		if (strVector.size() != 3) 
		{
			printf("PointCloud Type Is Not Supported: %s", file_path.c_str());
		}
		PointT p3d;
		p3d.x = stod(strVector[0]);
		p3d.y = stod(strVector[1]);
		p3d.z = stod(strVector[2]);
		read_cloud->push_back(p3d);
	}
	fin.close();
}

void loadEulerAngleFormTxt(const std::string &file_path, std::vector<Eigen::VectorXf> &anglesList)
{
	std::ifstream fin(file_path.c_str());
	if (!fin.is_open())
	{
		printf("Cannot Open File: %s", file_path.c_str());
	}
	std::string lineStr;
	while (getline(fin, lineStr))
	{
		Eigen::VectorXf eulerAngle(6);
		std::stringstream ss(lineStr);
		ss >> eulerAngle[0] >> eulerAngle[1] >> eulerAngle[2] >> eulerAngle[3] >> eulerAngle[4] >> eulerAngle[5];
		anglesList.push_back(eulerAngle);
	}
	fin.close();

}
void loadMatrixPointFormTxt(const std::string &file_path, Eigen::MatrixXf &cloud_matrix)
{
	struct Point
	{
		double x, y, z;
	};

	std::ifstream fin;
	fin.open(file_path.c_str(), std::ios::in);

	if (!fin.is_open())
	{
		printf("Cannot Open File: %s", file_path.c_str());
		exit(-1);
	}

	std::string lineStr;
	Point point;
	std::vector<Point> cloud;

	while (getline(fin, lineStr)) {
		if (lineStr.size() == 0) break;
		std::stringstream ss(lineStr);
		ss >> point.x >> point.y >> point.z;
		cloud.push_back(point);
	}
	cloud_matrix = Eigen::MatrixXf::Zero(cloud.size(), 3);
	for (int i = 0; i < cloud.size(); i++)
	{
		cloud_matrix(i, 0) = cloud[i].x;
		cloud_matrix(i, 1) = cloud[i].y;
		cloud_matrix(i, 2) = cloud[i].z;

	}
	fin.close();
}


/*-*-*-*-*-*-*-*-*-*-*-*- Cloud => Txt-*-*-*-*-*-*-*-*-*-*-*-*/
void savePointCloudToFile(const pcl::PointCloud<PointT>::Ptr write_cloud, const std::string file_path)
{
	std::ofstream fout;
	fout.open(file_path, std::ios::out);
	for (size_t i = 0; i < write_cloud->points.size(); i++)
	{
		fout << write_cloud->points[i].x << " "
			<< write_cloud->points[i].y << " "
			<< write_cloud->points[i].z << " "
			<< "\n";
	}
	fout.close();
}

/*-*-*-*-*-*-*-*-*-*-*-*- Txt => Matrix -*-*-*-*-*-*-*-*-*-*-*-*/
void loadMatrix4fFormTxt(const std::string &file_path, Eigen::Matrix4f &matrix)
{
	std::ifstream fin(file_path.c_str());
	if (!fin.is_open())
	{
		printf("Cannot Open File: %s", file_path.c_str());
		exit(-1);
	}

	std::string lineStr;
	for (size_t i = 0; getline(fin, lineStr); i++)
	{
		std::stringstream ss(lineStr);
		std::string valueStr;
		for (size_t j = 0; ss >> valueStr; j++)
		{
			std::string::size_type size;
			matrix(i, j) = stof(valueStr, &size);
		}
	}
	fin.close();
}


/*-*-*-*-*-*-*-*-*-*-*-*- Matrix => Txt-*-*-*-*-*-*-*-*-*-*-*-*/
void saveMatrixToFile(const Eigen::Matrix4f matrix, const std::string &file_path)
{
	std::ofstream fout;
	fout.open(file_path, std::ios::out);
	for (size_t r = 0; r < matrix.rows(); r++)
	{
		for (size_t c = 0; c < matrix.cols(); c++)
		{
			fout << matrix(r, c) << " ";
		}
		fout << std::endl;
	}

	fout.close();
}


