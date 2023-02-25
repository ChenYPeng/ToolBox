#pragma once
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/intersections.h>
#include <pcl/filters/extract_indices.h> //根据索引提取内点
#include <pcl/segmentation/sac_segmentation.h> //RANSAC分割

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>	// RANSAC拟合平面
#include <pcl/sample_consensus/sac_model_sphere.h>	// RANSAC拟合球
#include <pcl/sample_consensus/method_types.h>		//随机参数估计方法
#include <pcl/sample_consensus/model_types.h>		//模型定义

#include "utils.h"

void ConsSphereCloud(const Eigen::VectorXf &coefficient, pcl::PointCloud<PointT>::Ptr &cloud_ptr);

// (x-a)^2 + (y-b)^2 +(z-c)^2 = R^2
void sphereSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, Eigen::VectorXf &coefficient);

// Ax + By + Cz + D = 0
void planeSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients::Ptr &coefficients);
void mulityPlaneSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, int num_piece, std::vector<pcl::ModelCoefficients> &coeffs_list, std::vector<pcl::PointCloud<PointT>> &cloud_list);

//计算两平面交线
void getPlanerIntersectionLines(const pcl::ModelCoefficients coefsOfPlane1, const pcl::ModelCoefficients coefsOfPlane2, pcl::ModelCoefficients &coefsOfLine);

//计算两直线交点
void getStraightIntersections(const pcl::ModelCoefficients coefsOfLine1, const pcl::ModelCoefficients coefsOfLine2, Eigen::Vector4f &cross_point);


// (x-a)^2 + (y-b)^2 +(z-c)^2 = R^2
void sphereSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, Eigen::VectorXf &coefficient)
{
	//--------------------创建RANSAC对象并计算相应的模型--------------------------- 
	pcl::SampleConsensusModelSphere<PointT>::Ptr model_sphere(new pcl::SampleConsensusModelSphere<PointT>(cloud));

	pcl::RandomSampleConsensus<PointT> ransac(model_sphere);
	ransac.setDistanceThreshold(0.01); // 距离阈值
	ransac.computeModel();              // 拟合球面

	ransac.getModelCoefficients(coefficient);

	std::cout << "->球面方程为：\n"
		<< "(x - " << coefficient[0]
		<< ") ^ 2 + (y - " << coefficient[1]
		<< ") ^ 2 + (z - " << coefficient[2]
		<< ")^2 = " << coefficient[3]
		<< " ^2"
		<< std::endl;
}


// Ax + By + Cz + D = 0
void planeSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients::Ptr &coefficients)
{

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//索引列表
	pcl::SACSegmentation<PointT> sac;//分割对象

	sac.setInputCloud(cloud);
	sac.setOptimizeCoefficients(true);		//使用内部点重新估算模型参数
	sac.setModelType(pcl::SACMODEL_PLANE);	//设置模型类型
	sac.setMethodType(pcl::SAC_RANSAC);		//设置随机采样一致性方法类型
	sac.setDistanceThreshold(5.0f);		//设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
	sac.setMaxIterations(300);				//设置最大迭代次数
	sac.segment(*inliers, *coefficients);	//得到平面系数已经在平面上的点的索引

	std::cout << "->平面方程为：\n"
		<< coefficients->values[0] << "X + " 
		<< coefficients->values[1] << "Y + "
		<< coefficients->values[1] << "Z + "
		<< coefficients->values[1] << std::endl;
}


void mulityPlaneSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, int num_piece, std::vector<pcl::ModelCoefficients> &coeffs_list, std::vector<pcl::PointCloud<PointT>> &cloud_list)
{

	//第一步：定义输入的原始数据以及分割获得的点、平面系数coefficients、存储内点的索引集合对象inliers
	pcl::PointCloud<PointT>::Ptr plane_segment(new pcl::PointCloud<PointT>);//创建分割对象
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//模型系数
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//索引列表
	pcl::SACSegmentation<PointT> sac;//分割对象
	pcl::ExtractIndices<PointT> extract;//提取器

	cloud_list.reserve(num_piece);

	//第二步：使用RANSAC获取点数最多的面
	for (int i = 0; i < num_piece; i++)
	{

		sac.setInputCloud(cloud);
		sac.setOptimizeCoefficients(true);		//使用内部点重新估算模型参数
		sac.setModelType(pcl::SACMODEL_PLANE);	//设置模型类型
		sac.setMethodType(pcl::SAC_RANSAC);		//设置随机采样一致性方法类型
		sac.setDistanceThreshold(0.6f);			//设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
		sac.setMaxIterations(300);				//设置最大迭代次数
		sac.segment(*inliers, *coefficients);	//得到平面系数　已经在平面上的点的　索引
		coeffs_list.push_back(*coefficients);

		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*plane_segment);			//提取探测出来的平面
		extract.setNegative(true);				//剔除探测出的平面，在剩余点中继续探测平面
		extract.filter(*cloud);

		cloud_list.push_back(*plane_segment);	//保存该次分割的平面
	}
}

Eigen::Vector4d PlaneFitting(const std::vector<Eigen::Vector3d> & plane_pts)
{
	Eigen::Vector3d center = Eigen::Vector3d::Zero();
	for (const auto & pt : plane_pts) center += pt;
	center /= plane_pts.size();

	Eigen::MatrixXd A(plane_pts.size(), 3);
	for (int i = 0; i < plane_pts.size(); i++) {
		A(i, 0) = plane_pts[i][0] - center[0];
		A(i, 1) = plane_pts[i][1] - center[1];
		A(i, 2) = plane_pts[i][2] - center[2];
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
	const float a = svd.matrixV()(0, 2);
	const float b = svd.matrixV()(1, 2);
	const float c = svd.matrixV()(2, 2);
	const float d = -(a * center[0] + b * center[1] + c * center[2]);
	return Eigen::Vector4d(a, b, c, d);
}

//计算两平面交线
void getPlanerIntersectionLines(const pcl::ModelCoefficients coefsOfPlane1, const pcl::ModelCoefficients coefsOfPlane2, pcl::ModelCoefficients &coefsOfLine)
{
	//平面1法向量 n1=(a1, b1, c1)
	//平面2法向量 n2=(a2, b2, c2)
	//交线方向向量 n=n1×n2=(b1 * c2 - c1 * b2, c1 * a2 - a1 * c2, a1 * b2 - b1 * a2)
	pcl::ModelCoefficients temcoefs;
	double a1, b1, c1, d1, a2, b2, c2, d2;
	double tempy, tempz;

	a1 = coefsOfPlane1.values[0];
	b1 = coefsOfPlane1.values[1];
	c1 = coefsOfPlane1.values[2];
	d1 = coefsOfPlane1.values[3];
	a2 = coefsOfPlane2.values[0];
	b2 = coefsOfPlane2.values[1];
	c2 = coefsOfPlane2.values[2];
	d2 = coefsOfPlane2.values[3];

	tempz = -(d1 / b1 - d2 / b2) / (c1 / b1 - c2 / b2);
	tempy = (-c1 / b1)*tempz - d1 / b1;

	coefsOfLine.values.push_back(0.0);
	coefsOfLine.values.push_back(tempy);
	coefsOfLine.values.push_back(tempz);
	coefsOfLine.values.push_back(b1 * c2 - c1 * b2);
	coefsOfLine.values.push_back(c1 * a2 - a1 * c2);
	coefsOfLine.values.push_back(a1 * b2 - b1 * a2);
}

//计算两直线交点
void getStraightIntersections(const pcl::ModelCoefficients coefsOfLine1, const pcl::ModelCoefficients coefsOfLine2, Eigen::Vector4f &cross_point)
{

	pcl::lineWithLineIntersection(coefsOfLine1, coefsOfLine2, cross_point);

}

//构造球体点云 
void ConsSphereCloud(const Eigen::VectorXf &coefficient, pcl::PointCloud<PointT>::Ptr &cloud_ptr)
{
	/*
		x = R*sin(θ)*cos(ψ);
		y = R*sin(θ)*sin(ψ);  θ∈[0, PI), ψ∈[0, 2*PI)
		z = R*cos(θ);
	*/
	PointT center(coefficient[0], coefficient[1], coefficient[2]);

	float radius = coefficient[3];

	for (float angle1 = 0.0; angle1 <= 180.0; angle1 += 1.0)
	{
		for (float angle2 = 0.0; angle2 <= 360.0; angle2 += 1.0)
		{
			PointT tempPoint;

			tempPoint.x = center.x + radius * sinf(pcl::deg2rad(angle1)) * cosf(pcl::deg2rad(angle2));
			tempPoint.y = center.y + radius * sinf(pcl::deg2rad(angle1)) * sinf(pcl::deg2rad(angle2));
			tempPoint.z = center.z + radius * cosf(pcl::deg2rad(angle1));
			cloud_ptr->points.push_back(tempPoint);
		}
	}

	cloud_ptr->width = (int)cloud_ptr->points.size();
	cloud_ptr->height = 1;


}

//构造圆柱体点云  
void ConsCylinderCloud(const Eigen::VectorXf &coefficient, pcl::PointCloud<PointT>::Ptr &cloud_ptr)
{
	/*
		x = R * cos(θ);
		y = R * sin(θ);  θ∈[-2*PI, 2*PI), z∈(-∞，+∞）
		z = z;
	*/
	for (float z = -1.0; z <= 1.0; z += 0.05)
	{
		for (float angle = 0.0; angle <= 360.0; angle += 5.0)
		{
			PointT tempPoint;
			tempPoint.x = cosf(pcl::deg2rad(angle));
			tempPoint.y = sinf(pcl::deg2rad(angle));
			tempPoint.z = z;
			cloud_ptr->points.push_back(tempPoint);
		}
	}

	cloud_ptr->width = (int)cloud_ptr->points.size();
	cloud_ptr->height = 1;
}