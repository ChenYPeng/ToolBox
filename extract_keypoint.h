#pragma once
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3D.h>
#include <pcl/keypoints/sift_keypoint.h>

#include "utils.h"
#include "extract_keypoint.h"

void compute3DCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ &center);

void demeanPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ center, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

// 计算点云分辨率
double computeCloudResolution(const pcl::PointCloud<PointT>::Ptr& cloud);

// 点云法向量估计
void estimateNormals(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals);

// 拼接点云与法线信息
void cloud_with_normal(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointNormalT>::Ptr& source_with_normals);

void compute_fpfh(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &keypoints);

//void compute_sift(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &keypoints);

//void compute_harris(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &keypoints);

//void compute_iss(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &keypoints);


void compute3DCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ &center)
{
	// 按公式计算质心
	center.x = 0; center.y = 0; center.z = 0;
	for (auto p : cloud->points) {
		center.x += p.x;
		center.y += p.y;
		center.z += p.z;
	}

	center.x /= cloud->points.size();
	center.y /= cloud->points.size();
	center.z /= cloud->points.size();

}

void demeanPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ center, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	// 归一化
	for (auto p : cloud_in->points) {

		pcl::PointXYZ p_mean;
		p_mean.x = p.x - center.x;
		p_mean.y = p.y - center.y;
		p_mean.z = p.z - center.z;
		cloud_out->points.push_back(p_mean);
	}

}

// 计算点云分辨率
double computeCloudResolution(const pcl::PointCloud<PointT>::Ptr& cloud)
{
	/*
	step1: 遍历点云中的每一个点
	step2: 找到与其最近的点，并求出两点距离
	step3: 累加两点距离之和，并除以点云数量（剔除无效点），所得结果便是点云空间分辨率
	*/
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);

	//step1: 新建kdtree用于搜索
	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud(cloud);

	//step2: 遍历点云每个点，并找出与它距离最近的点
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
			continue;

		// 取第二个距离，因为第一个是它本身
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		//step3: 统计最小距离和、有效点数量
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	//step4: 计算空间分辨率
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}

// 点云法向量估计
void estimateNormals(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
{
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud(source_cloud); //设置法线估计输入点云

	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(source_cloud);

	ne.setSearchMethod(tree);
	// 两种s搜索方式二选一
	ne.setKSearch(30);				//点云法向计算时，需要所搜的近邻点大小
	//ne.setRadiusSearch(2);		//设置半径邻域的大小

	//开始进行法向计算
	ne.compute(*cloud_normals);

}

// 拼接点云与法线信息
void cloud_with_normal(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointNormalT>::Ptr& source_with_normals)
{

	//执行法线估计，并将结果保存到cloud_normals中
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	estimateNormals(source_cloud, cloud_normals);

	//将点云数据与法向信息拼接
	pcl::concatenateFields(*source_cloud, *cloud_normals, *source_with_normals);

}

void compute_fpfh(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &keypoints)
{
	pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud(source_cloud);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	estimateNormals(source_cloud, cloud_normals);
	fpfh_est.setInputNormals(cloud_normals);

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	fpfh_est.setSearchMethod(tree);

	fpfh_est.setKSearch(30);
	//fpfh_est.setRadiusSearch(3);
	fpfh_est.compute(*keypoints);


}

//void compute_sift(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &keypoints)
//{
//	//设置 sift 相关参数
//	const float min_scale = 0.01;             //设置尺度空间中最小尺度的标准偏差          
//	const int n_octaves = 6;                  //设置高斯金字塔组（octave）的数目            
//	const int n_scales_per_octave = 4;        //设置每组（octave）计算的尺度  
//	const float min_contrast = 0.01;          //设置限制关键点检测的阈值
//
//	pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_det;
//	sift_det.setInputCloud(cloud);
//
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());//声明一个Kd-tree
//	sift_det.setSearchMethod(tree);
//
//	/*指定搜索关键点的尺度范围*/
//	sift_det.setScales(min_scale, n_octaves, n_scales_per_octave);
//
//	/*设置限制关键点检测的阈值ֵ*/
//	sift_det.setMinimumContrast(min_contrast);
//
//	/*执行sift关键点检测，保存结果在result*/
//	pcl::PointCloud<pcl::PointWithScale>::Ptr sift_keypoints(new pcl::PointCloud<pcl::PointWithScale>);
//	sift_det.compute(*sift_keypoints);
//
//	pcl::copyPointCloud(*sift_keypoints, *keypoints);
//
//}

//void compute_harris(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &keypoints)
//{
//
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//	normal_estimation(cloud, cloud_normals);
//
//	pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI, pcl::Normal> harris_det;
//	harris_det.setInputCloud(cloud);
//
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//	harris_det.setSearchMethod(tree);
//	harris_det.setMethod(harris_det.LOWE);	//设置要计算响应的方法（可以不设置）
//	harris_det.setNormals(cloud_normals);	//输入法向量
//	harris_det.setRadius(3);				//设置法线估计和非极大值抑制的半径。
//	harris_det.setRadiusSearch(3); 			//设置用于关键点检测的最近邻居的球半径
//	harris_det.setNonMaxSupression(true);	//是否进行R阈值检测，给定范围内的非极大值抑制（这个给定的范围跟求R值时使用到的球半径是一样的），无穷大响应值筛除
//	harris_det.setRefine(true);				//是否进行细化，细化后提取出来的点一定为点云中的点
//	harris_det.setNumberOfThreads(6);		//初始化调度程序并设置要使用的线程数
//	harris_det.setThreshold(1e-6);			//设置角点检测阈值，只有当非极大值抑制设置为true时才有效
//
//	pcl::PointCloud<pcl::PointXYZI>::Ptr Harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>);
//	harris_det.compute(*Harris_keypoints);
//
//	pcl::PointIndicesConstPtr keypoints_indices = harris_det.getKeypointsIndices();
//	pcl::copyPointCloud(*cloud, *keypoints_indices, *keypoints);
//}


//void compute_iss(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &keypoints)
//{
//
//	// 计算关键点
//	pcl::ISSKeypoint3D<PointT, PointT> iss_det;
//	iss_det.setInputCloud(cloud);
//
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//	iss_det.setSearchMethod(tree);
//
//	// 计算分辨率
//	double model_resolution = computeCloudResolution(cloud);
//
//	//iss公共参数设置
//	iss_det.setMinNeighbors(20);
//	iss_det.setThreshold21(0.975);
//	iss_det.setThreshold32(0.975);
//	iss_det.setNumberOfThreads(4);
//
//	// 计算model关键点
//	iss_det.setSalientRadius(6 * model_resolution);
//	iss_det.setNonMaxRadius(4 * model_resolution);
//	iss_det.compute(*keypoints);
//}