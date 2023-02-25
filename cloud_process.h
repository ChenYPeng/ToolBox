#pragma once
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/intersections.h>
#include <pcl/filters/extract_indices.h> //����������ȡ�ڵ�
#include <pcl/segmentation/sac_segmentation.h> //RANSAC�ָ�

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>	// RANSAC���ƽ��
#include <pcl/sample_consensus/sac_model_sphere.h>	// RANSAC�����
#include <pcl/sample_consensus/method_types.h>		//����������Ʒ���
#include <pcl/sample_consensus/model_types.h>		//ģ�Ͷ���

#include "utils.h"

void ConsSphereCloud(const Eigen::VectorXf &coefficient, pcl::PointCloud<PointT>::Ptr &cloud_ptr);

// (x-a)^2 + (y-b)^2 +(z-c)^2 = R^2
void sphereSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, Eigen::VectorXf &coefficient);

// Ax + By + Cz + D = 0
void planeSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients::Ptr &coefficients);
void mulityPlaneSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, int num_piece, std::vector<pcl::ModelCoefficients> &coeffs_list, std::vector<pcl::PointCloud<PointT>> &cloud_list);

//������ƽ�潻��
void getPlanerIntersectionLines(const pcl::ModelCoefficients coefsOfPlane1, const pcl::ModelCoefficients coefsOfPlane2, pcl::ModelCoefficients &coefsOfLine);

//������ֱ�߽���
void getStraightIntersections(const pcl::ModelCoefficients coefsOfLine1, const pcl::ModelCoefficients coefsOfLine2, Eigen::Vector4f &cross_point);


// (x-a)^2 + (y-b)^2 +(z-c)^2 = R^2
void sphereSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, Eigen::VectorXf &coefficient)
{
	//--------------------����RANSAC���󲢼�����Ӧ��ģ��--------------------------- 
	pcl::SampleConsensusModelSphere<PointT>::Ptr model_sphere(new pcl::SampleConsensusModelSphere<PointT>(cloud));

	pcl::RandomSampleConsensus<PointT> ransac(model_sphere);
	ransac.setDistanceThreshold(0.01); // ������ֵ
	ransac.computeModel();              // �������

	ransac.getModelCoefficients(coefficient);

	std::cout << "->���淽��Ϊ��\n"
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

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//�����б�
	pcl::SACSegmentation<PointT> sac;//�ָ����

	sac.setInputCloud(cloud);
	sac.setOptimizeCoefficients(true);		//ʹ���ڲ������¹���ģ�Ͳ���
	sac.setModelType(pcl::SACMODEL_PLANE);	//����ģ������
	sac.setMethodType(pcl::SAC_RANSAC);		//�����������һ���Է�������
	sac.setDistanceThreshold(5.0f);		//�趨���뷧ֵ�����뷧ֵ�����˵㱻��Ϊ�Ǿ��ڵ��Ǳ������������
	sac.setMaxIterations(300);				//��������������
	sac.segment(*inliers, *coefficients);	//�õ�ƽ��ϵ���Ѿ���ƽ���ϵĵ������

	std::cout << "->ƽ�淽��Ϊ��\n"
		<< coefficients->values[0] << "X + " 
		<< coefficients->values[1] << "Y + "
		<< coefficients->values[1] << "Z + "
		<< coefficients->values[1] << std::endl;
}


void mulityPlaneSegRansac(const pcl::PointCloud<PointT>::Ptr cloud, int num_piece, std::vector<pcl::ModelCoefficients> &coeffs_list, std::vector<pcl::PointCloud<PointT>> &cloud_list)
{

	//��һ�������������ԭʼ�����Լ��ָ��õĵ㡢ƽ��ϵ��coefficients���洢�ڵ���������϶���inliers
	pcl::PointCloud<PointT>::Ptr plane_segment(new pcl::PointCloud<PointT>);//�����ָ����
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//ģ��ϵ��
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//�����б�
	pcl::SACSegmentation<PointT> sac;//�ָ����
	pcl::ExtractIndices<PointT> extract;//��ȡ��

	cloud_list.reserve(num_piece);

	//�ڶ�����ʹ��RANSAC��ȡ����������
	for (int i = 0; i < num_piece; i++)
	{

		sac.setInputCloud(cloud);
		sac.setOptimizeCoefficients(true);		//ʹ���ڲ������¹���ģ�Ͳ���
		sac.setModelType(pcl::SACMODEL_PLANE);	//����ģ������
		sac.setMethodType(pcl::SAC_RANSAC);		//�����������һ���Է�������
		sac.setDistanceThreshold(0.6f);			//�趨���뷧ֵ�����뷧ֵ�����˵㱻��Ϊ�Ǿ��ڵ��Ǳ������������
		sac.setMaxIterations(300);				//��������������
		sac.segment(*inliers, *coefficients);	//�õ�ƽ��ϵ�����Ѿ���ƽ���ϵĵ�ġ�����
		coeffs_list.push_back(*coefficients);

		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*plane_segment);			//��ȡ̽�������ƽ��
		extract.setNegative(true);				//�޳�̽�����ƽ�棬��ʣ����м���̽��ƽ��
		extract.filter(*cloud);

		cloud_list.push_back(*plane_segment);	//����ôηָ��ƽ��
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

//������ƽ�潻��
void getPlanerIntersectionLines(const pcl::ModelCoefficients coefsOfPlane1, const pcl::ModelCoefficients coefsOfPlane2, pcl::ModelCoefficients &coefsOfLine)
{
	//ƽ��1������ n1=(a1, b1, c1)
	//ƽ��2������ n2=(a2, b2, c2)
	//���߷������� n=n1��n2=(b1 * c2 - c1 * b2, c1 * a2 - a1 * c2, a1 * b2 - b1 * a2)
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

//������ֱ�߽���
void getStraightIntersections(const pcl::ModelCoefficients coefsOfLine1, const pcl::ModelCoefficients coefsOfLine2, Eigen::Vector4f &cross_point)
{

	pcl::lineWithLineIntersection(coefsOfLine1, coefsOfLine2, cross_point);

}

//����������� 
void ConsSphereCloud(const Eigen::VectorXf &coefficient, pcl::PointCloud<PointT>::Ptr &cloud_ptr)
{
	/*
		x = R*sin(��)*cos(��);
		y = R*sin(��)*sin(��);  �ȡ�[0, PI), �ס�[0, 2*PI)
		z = R*cos(��);
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

//����Բ�������  
void ConsCylinderCloud(const Eigen::VectorXf &coefficient, pcl::PointCloud<PointT>::Ptr &cloud_ptr)
{
	/*
		x = R * cos(��);
		y = R * sin(��);  �ȡ�[-2*PI, 2*PI), z��(-�ޣ�+�ޣ�
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