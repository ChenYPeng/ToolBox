#pragma once
#include <Eigen/Core>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h> //随机采样一致性配准
#include <pcl/registration/transformation_estimation_point_to_plane.h>

#include "utils.h"
#include "extract_keypoint.h"
#include "outlier_removal.h"


//SAC_IA粗配准
void sac_ia_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f& tf,
	int max_sacia_iterations = 200, double min_correspondence_dist = 1.0, double max_correspondence_dist = 40.0);

//NDT粗配准
void ndt_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf,
	int max_sacia_iterations = 200, double step_size = 2.0, float resolution = 1.0);

//4FPCS粗配准
void fpcs_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf);

//RANSAC粗配准
void ransac_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf);

// 精配准
void ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf);

void KD_ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf);

void SGI_ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf);

void N_ICP(const pcl::PointCloud<PointNormalT>::Ptr source, const pcl::PointCloud<PointNormalT>::Ptr target, Eigen::Matrix4f& tf);

void lm_ICP(const pcl::PointCloud<PointNormalT>::Ptr source_with_normals, const pcl::PointCloud<PointNormalT>::Ptr target_with_normals, Eigen::Matrix4f& tf, float distance=3.0);

void pairAlign(const pcl::PointCloud<PointT>::Ptr cloud_src, const pcl::PointCloud<PointT>::Ptr cloud_tgt, Eigen::Matrix4f &final_transform, bool downsample = false);


/*
SVD求解三维变换矩阵Rt（旋转+平移）:
	P={p1,p2,...,pn}和Q={q1,q2,...,qn}是两组空间中的对应点集
	P或Q中的样点不能共线

*/
void pointSetRegistrationUmeyama(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf);
void pointCloudRegistrationSVD(const pcl::PointCloud<PointT>::Ptr Psource, const pcl::PointCloud<PointT>::Ptr Ptarget, Eigen::Matrix4f &Hx);
void pointSetRegistrationSVD(const std::vector<Eigen::Vector3f>p1, const std::vector<Eigen::Vector3f>p2, Eigen::Matrix4f &Hx);

//sac粗配准
void sac_ia_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f& tf,
	int max_sacia_iterations, double min_correspondence_dist, double max_correspondence_dist)
{

	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source_cloud);
	sac_ia.setInputTarget(target_cloud);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_source(new pcl::PointCloud<pcl::FPFHSignature33>);
	compute_fpfh(source_cloud, fpfhs_source);
	sac_ia.setSourceFeatures(fpfhs_source);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_target(new pcl::PointCloud<pcl::FPFHSignature33>);
	compute_fpfh(target_cloud, fpfhs_target);
	sac_ia.setTargetFeatures(fpfhs_target);

	sac_ia.setMaximumIterations(max_sacia_iterations); // 100
	sac_ia.setMinSampleDistance(min_correspondence_dist); // 1
	sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist); //20

	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	sac_ia.align(*Final);
	tf = sac_ia.getFinalTransformation().cast<float>();
}

//NDT粗配准
void ndt_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf,
	int max_sacia_iterations, double step_size, float resolution)
{
	//初始化正态分布变换（NDT）
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;
	ndt.setTransformationEpsilon(1e-3);					// 为终止条件设置最小转换差异
	ndt.setStepSize(step_size);							// 为More-Thuente线搜索设置最大步长
	ndt.setResolution(resolution);						// 设置NDT网格结构的分辨率（VoxelGridCovariance）（体素格的大小）
	ndt.setMaximumIterations(max_sacia_iterations);		// 设置匹配迭代的最大次数
	ndt.setInputSource(source_cloud);					// 设置要配准的点云
	ndt.setInputTarget(target_cloud);					// 设置点云配准目标

	//计算需要的刚体变换以便将输入的点云匹配到目标点云
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	ndt.align(*Final);

	tf = ndt.getFinalTransformation().cast<float>();
}

//4FPCS粗配准
void fpcs_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf)
{

	//初始化正态分布变换（NDT）
	pcl::registration::FPCSInitialAlignment<PointT, PointT> fpcs;
	fpcs.setInputSource(source_cloud);  // 源点云
	fpcs.setInputTarget(target_cloud);  // 目标点云
	fpcs.setApproxOverlap(0.1);          // 设置源和目标之间的近似重叠度。
	fpcs.setDelta(0.01);                // 设置配准后对应点之间的距离（以米为单位）。
	fpcs.setNumberOfSamples(100);       // 设置验证配准效果时要使用的采样点数量

	//计算需要的刚体变换以便将输入的点云匹配到目标点云
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	fpcs.align(*Final);

	tf = fpcs.getFinalTransformation().cast<float>();
}

//RANSAC粗配准
void ransac_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf)
{

	pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> ransac;
	ransac.setInputSource(source_cloud);		// 源点云
	ransac.setInputTarget(target_cloud);     // 目标点云

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_source(new pcl::PointCloud<pcl::FPFHSignature33>);
	compute_fpfh(source_cloud, fpfhs_source);
	ransac.setSourceFeatures(fpfhs_source);   // 源点云FPFH特征

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_target(new pcl::PointCloud<pcl::FPFHSignature33>);
	compute_fpfh(target_cloud, fpfhs_target);
	ransac.setTargetFeatures(fpfhs_target);    // 目标点云FPFH特征

	ransac.setCorrespondenceRandomness(5);      // 在选择随机特征对应时，设置要使用的邻居的数量,数值越大，特征匹配的随机性越大。
	ransac.setInlierFraction(0.5f);             // 所需的(输入的)inlier分数
	ransac.setNumberOfSamples(3);               // 每次迭代中使用的采样点数量
	ransac.setSimilarityThreshold(0.1f);        // 将底层多边形对应拒绝器对象的边缘长度之间的相似阈值设置为[0,1]，其中1为完全匹配。
	ransac.setMaxCorrespondenceDistance(1.0f);  // 内点，阈值 Inlier threshold
	ransac.setMaximumIterations(1000);           // RANSAC 　最大迭代次数

	//计算需要的刚体变换以便将输入的点云匹配到目标点云
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	ransac.align(*Final);

	tf = ransac.getFinalTransformation().cast<float>();
}


// 精配准

void ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf) {

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	//----------------------icp核心代码--------------------
	icp.setInputSource(source);						// 源点云
	icp.setInputTarget(target);						// 目标点云
	icp.setTransformationEpsilon(1e-6);             // 为终止条件设置最小转换差异
	icp.setMaxCorrespondenceDistance(3);          // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
	icp.setEuclideanFitnessEpsilon(1e-6);           // 设置收敛条件是均方误差和小于阈值， 停止迭代；
	icp.setMaximumIterations(1250);                   // 最大迭代次数
	icp.setRANSACOutlierRejectionThreshold(0.05);   // 剔除错误估计，可用 RANSAC 算法，或减少数量
	//icp.setUseReciprocalCorrespondences(true);    // 设置为true,则使用相互对应关系

	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	icp.align(*Final);

	tf = icp.getFinalTransformation().cast<float>();
}

void KD_ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf) {

	pcl::IterativeClosestPoint<PointT, PointT> icp;

	//---------------------KD树加速搜索--------------------
	pcl::search::KdTree<PointT>::Ptr source_tree(new pcl::search::KdTree<PointT>);
	source_tree->setInputCloud(source);
	pcl::search::KdTree<PointT>::Ptr target_tree(new pcl::search::KdTree<PointT>);
	target_tree->setInputCloud(target);
	icp.setSearchMethodSource(source_tree);
	icp.setSearchMethodTarget(target_tree);

	//----------------------icp核心代码--------------------
	icp.setInputSource(source);						// 源点云
	icp.setInputTarget(target);						// 目标点云
	icp.setTransformationEpsilon(1e-10);             // 为终止条件设置最小转换差异
	icp.setMaxCorrespondenceDistance(10);          // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
	icp.setEuclideanFitnessEpsilon(1e-3);           // 设置收敛条件是均方误差和小于阈值， 停止迭代；
	icp.setMaximumIterations(250);                   // 最大迭代次数
	icp.setRANSACOutlierRejectionThreshold(0.05);   // 剔除错误估计，可用 RANSAC 算法，或减少数量
	//icp.setUseReciprocalCorrespondences(true);    // 设置为true,则使用相互对应关系

	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	icp.align(*Final);

	tf = icp.getFinalTransformation().cast<float>();
}

void SGI_ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf) {

	pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;

	//---------------------KD树加速搜索--------------------
	pcl::search::KdTree<PointT>::Ptr source_tree(new pcl::search::KdTree<PointT>);
	source_tree->setInputCloud(source);
	pcl::search::KdTree<PointT>::Ptr target_tree(new pcl::search::KdTree<PointT>);
	target_tree->setInputCloud(target);
	gicp.setSearchMethodSource(source_tree);
	gicp.setSearchMethodTarget(target_tree);

	//-----------------设置GICP相关参数-----------------------
	gicp.setInputSource(source);  //源点云
	gicp.setInputTarget(target);  //目标点云
	gicp.setMaxCorrespondenceDistance(1.0); //设置对应点对之间的最大距离
	gicp.setTransformationEpsilon(1e-10);   //为终止条件设置最小转换差异
	gicp.setEuclideanFitnessEpsilon(1e-3);  //设置收敛条件是均方误差和小于阈值， 停止迭代
	gicp.setMaximumIterations(35); //最大迭代次数  
	//gicp.setUseReciprocalCorrespondences(true);//使用相互对应关系

	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	gicp.align(*Final);

	tf = gicp.getFinalTransformation().cast<float>();
}


void N_ICP(const pcl::PointCloud<PointNormalT>::Ptr source, const pcl::PointCloud<PointNormalT>::Ptr target, Eigen::Matrix4f& tf)
{

	pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT>n_icp;

	n_icp.setInputSource(source);
	n_icp.setInputTarget(target);
	n_icp.setTransformationEpsilon(1e-10);		// 为终止条件设置最小转换差异
	n_icp.setMaxCorrespondenceDistance(2);	// 设置对应点对之间的最大距离（此值对配准结果影响较大）。
	n_icp.setEuclideanFitnessEpsilon(1e-6);		// 设置收敛条件是均方误差和小于阈值， 停止迭代；
	n_icp.setUseReciprocalCorrespondences(false);// 设置为true则变为另一个算法
	n_icp.setMaximumIterations(150);				// 最大迭代次数

	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<PointNormalT>::Ptr Final(new pcl::PointCloud<PointNormalT>);
	n_icp.align(*Final);

	tf = n_icp.getFinalTransformation().cast<float>();
}


// 点到面ICP(非线性最小二乘优化)
void lm_ICP(const pcl::PointCloud<PointNormalT>::Ptr source_with_normals, const pcl::PointCloud<PointNormalT>::Ptr target_with_normals,
	Eigen::Matrix4f& tf, float distance)
{
	pcl::IterativeClosestPoint<PointNormalT, PointNormalT>lm_icp;
	pcl::registration::TransformationEstimationPointToPlane<PointNormalT, PointNormalT>::Ptr PointToPlane
	(new pcl::registration::TransformationEstimationPointToPlane<PointNormalT, PointNormalT>);
	lm_icp.setTransformationEstimation(PointToPlane);
	lm_icp.setInputSource(source_with_normals);
	lm_icp.setInputTarget(target_with_normals);
	lm_icp.setTransformationEpsilon(1e-10);				// 为终止条件设置最小转换差异
	lm_icp.setMaxCorrespondenceDistance(distance);		// 设置对应点对之间的最大距离（此值对配准结果影响较大）。
	lm_icp.setEuclideanFitnessEpsilon(1e-4);			// 设置收敛条件是均方误差和小于阈值， 停止迭代；
	lm_icp.setMaximumIterations(500);					 // 最大迭代次数
	lm_icp.setUseReciprocalCorrespondences(false);		//使用相互对应关系

	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<PointNormalT>::Ptr Final(new pcl::PointCloud<PointNormalT>);
	lm_icp.align(*Final);

	tf = lm_icp.getFinalTransformation().cast<float>();

}

void pairAlign(const pcl::PointCloud<PointT>::Ptr cloud_src, const pcl::PointCloud<PointT>::Ptr cloud_tgt, Eigen::Matrix4f &final_transform, bool downsample)
{
	pcl::PointCloud<PointT>::Ptr src(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr tgt(new pcl::PointCloud<PointT>);

	// 是否先对点云进行下采样
	if (downsample)
	{
		downSampleVoxelGrids(cloud_src, src, 2.5, 2.5, 2.5);
		downSampleVoxelGrids(cloud_tgt, tgt, 2.5, 2.5, 2.5);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}

	//计算曲面法线和曲率
	pcl::PointCloud<PointNormalT>::Ptr points_with_normals_src(new pcl::PointCloud<PointNormalT>);
	cloud_with_normal(src, points_with_normals_src);

	pcl::PointCloud<PointNormalT>::Ptr points_with_normals_tgt(new pcl::PointCloud<PointNormalT>);
	cloud_with_normal(tgt, points_with_normals_tgt);

	// 点云配准
	lm_ICP(points_with_normals_src, points_with_normals_tgt, final_transform);

}


// 点集配准
void pointSetRegistrationUmeyama(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & TransRT)
{
	assert(source_cloud->points.size() != target_cloud->points.size());
	int pointNum = source_cloud->points.size();

	Eigen::Matrix<float, 3, Eigen::Dynamic> source_matrix(3, pointNum);
	Eigen::Matrix<float, 3, Eigen::Dynamic> target_matrix(3, pointNum);

	for (int i = 0; i < pointNum; i++)
	{
		source_matrix(0, i) = source_cloud->points[i].x;
		source_matrix(1, i) = source_cloud->points[i].y;
		source_matrix(2, i) = source_cloud->points[i].z;

		target_matrix(0, i) = target_cloud->points[i].x;
		target_matrix(1, i) = target_cloud->points[i].y;
		target_matrix(2, i) = target_cloud->points[i].z;
	}
	TransRT = Eigen::umeyama(source_matrix, target_matrix, true).cast<float>();
}


void pointCloudRegistrationSVD(const pcl::PointCloud<PointT>::Ptr Psource,
	const pcl::PointCloud<PointT>::Ptr Ptarget,
	Eigen::Matrix4f &Hx)
{
	assert(Psource->points.size() == Ptarget->points.size());

	// center of mass
	PointT ps_center, pt_center;
	compute3DCenter(Psource, ps_center);
	compute3DCenter(Ptarget, pt_center);

	// remove the center
	pcl::PointCloud<PointT>::Ptr mean_Psource(new pcl::PointCloud<PointT>);
	demeanPointCloud(Psource, ps_center, mean_Psource);
	pcl::PointCloud<PointT>::Ptr mean_Ptarget(new pcl::PointCloud<PointT>);
	demeanPointCloud(Ptarget, pt_center, mean_Ptarget);

	// compute W = q2*q1^T
	Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
	for (size_t i = 0; i < mean_Psource->points.size(); i++)
	{
		W += Eigen::Vector3f(mean_Ptarget->points[i].x, mean_Ptarget->points[i].y, mean_Ptarget->points[i].z) *
			Eigen::Vector3f(mean_Psource->points[i].x, mean_Psource->points[i].y, mean_Psource->points[i].z).transpose();

	}
	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullV | Eigen::ComputeFullU);
	Eigen::MatrixXf V = svd.matrixV();
	Eigen::MatrixXf U = svd.matrixU();
	double determinant = W.determinant();
	Eigen::Matrix3f R_ans = Eigen::Matrix3f::Zero();
	// 若行列式未负，则R的最优值要取负
	if (determinant < 0) {
		R_ans = -U * V.transpose();
	}
	else {
		R_ans = U * V.transpose();
	}
	// 最后根据求出的R，解t
	Eigen::MatrixXf t_ans = Eigen::Vector3f(pt_center.x, pt_center.y, pt_center.z) -
		R_ans * Eigen::Vector3f(ps_center.x, ps_center.y, ps_center.z);

	Hx = Eigen::Matrix4f::Identity();

	Hx.block<3, 3>(0, 0) = R_ans;
	Hx.block<3, 1>(0, 3) = t_ans;

}

void pointSetRegistrationSVD(const std::vector<Eigen::Vector3f>p1,
	const std::vector<Eigen::Vector3f>p2,
	Eigen::Matrix4f &Hx)
{
	assert(p1.size() == p2.size());

	// center of mass
	size_t N = p1.size();
	Eigen::Vector3f p1_center, p2_center;
	for (int i = 0; i < N; ++i) {
		p1_center += p1.at(i);
		p2_center += p2.at(i);
	}
	p1_center /= N;
	p2_center /= N;

	// remove the center
	std::vector<Eigen::Vector3f> q1(N), q2(N);
	for (int i = 0; i < N; ++i) {
		q1[i] = p1.at(i) - p1_center;
		q2[i] = p2.at(i) - p2_center;
	}

	// compute q2*q1^T
	Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
	for (int i = 0; i < N; ++i) {
		H += q2.at(i) * q1.at(i).transpose();
	}

	// SVD on H
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3f U = svd.matrixU();
	Eigen::Matrix3f V = svd.matrixV();

	Eigen::Matrix3f R_12 = V * U.transpose();
	if (R_12.determinant() < 0) {
		R_12 = -R_12;
	}

	Eigen::Vector3f t_12 = p1_center - R_12 * p2_center;

	Hx = Eigen::Matrix4f::Identity();

	Hx.block<3, 3>(0, 0) = R_12;
	Hx.block<3, 1>(0, 3) = t_12;
}