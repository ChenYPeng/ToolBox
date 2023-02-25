#pragma once
#include <Eigen/Core>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h> //�������һ������׼
#include <pcl/registration/transformation_estimation_point_to_plane.h>

#include "utils.h"
#include "extract_keypoint.h"
#include "outlier_removal.h"


//SAC_IA����׼
void sac_ia_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f& tf,
	int max_sacia_iterations = 200, double min_correspondence_dist = 1.0, double max_correspondence_dist = 40.0);

//NDT����׼
void ndt_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf,
	int max_sacia_iterations = 200, double step_size = 2.0, float resolution = 1.0);

//4FPCS����׼
void fpcs_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf);

//RANSAC����׼
void ransac_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf);

// ����׼
void ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf);

void KD_ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf);

void SGI_ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf);

void N_ICP(const pcl::PointCloud<PointNormalT>::Ptr source, const pcl::PointCloud<PointNormalT>::Ptr target, Eigen::Matrix4f& tf);

void lm_ICP(const pcl::PointCloud<PointNormalT>::Ptr source_with_normals, const pcl::PointCloud<PointNormalT>::Ptr target_with_normals, Eigen::Matrix4f& tf, float distance=3.0);

void pairAlign(const pcl::PointCloud<PointT>::Ptr cloud_src, const pcl::PointCloud<PointT>::Ptr cloud_tgt, Eigen::Matrix4f &final_transform, bool downsample = false);


/*
SVD�����ά�任����Rt����ת+ƽ�ƣ�:
	P={p1,p2,...,pn}��Q={q1,q2,...,qn}������ռ��еĶ�Ӧ�㼯
	P��Q�е����㲻�ܹ���

*/
void pointSetRegistrationUmeyama(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf);
void pointCloudRegistrationSVD(const pcl::PointCloud<PointT>::Ptr Psource, const pcl::PointCloud<PointT>::Ptr Ptarget, Eigen::Matrix4f &Hx);
void pointSetRegistrationSVD(const std::vector<Eigen::Vector3f>p1, const std::vector<Eigen::Vector3f>p2, Eigen::Matrix4f &Hx);

//sac����׼
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

	// ������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	sac_ia.align(*Final);
	tf = sac_ia.getFinalTransformation().cast<float>();
}

//NDT����׼
void ndt_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf,
	int max_sacia_iterations, double step_size, float resolution)
{
	//��ʼ����̬�ֲ��任��NDT��
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;
	ndt.setTransformationEpsilon(1e-3);					// Ϊ��ֹ����������Сת������
	ndt.setStepSize(step_size);							// ΪMore-Thuente������������󲽳�
	ndt.setResolution(resolution);						// ����NDT����ṹ�ķֱ��ʣ�VoxelGridCovariance�������ظ�Ĵ�С��
	ndt.setMaximumIterations(max_sacia_iterations);		// ����ƥ�������������
	ndt.setInputSource(source_cloud);					// ����Ҫ��׼�ĵ���
	ndt.setInputTarget(target_cloud);					// ���õ�����׼Ŀ��

	//������Ҫ�ĸ���任�Ա㽫����ĵ���ƥ�䵽Ŀ�����
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	ndt.align(*Final);

	tf = ndt.getFinalTransformation().cast<float>();
}

//4FPCS����׼
void fpcs_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf)
{

	//��ʼ����̬�ֲ��任��NDT��
	pcl::registration::FPCSInitialAlignment<PointT, PointT> fpcs;
	fpcs.setInputSource(source_cloud);  // Դ����
	fpcs.setInputTarget(target_cloud);  // Ŀ�����
	fpcs.setApproxOverlap(0.1);          // ����Դ��Ŀ��֮��Ľ����ص��ȡ�
	fpcs.setDelta(0.01);                // ������׼���Ӧ��֮��ľ��루����Ϊ��λ����
	fpcs.setNumberOfSamples(100);       // ������֤��׼Ч��ʱҪʹ�õĲ���������

	//������Ҫ�ĸ���任�Ա㽫����ĵ���ƥ�䵽Ŀ�����
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	fpcs.align(*Final);

	tf = fpcs.getFinalTransformation().cast<float>();
}

//RANSAC����׼
void ransac_icp(const pcl::PointCloud<PointT>::Ptr source_cloud, const pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f & tf)
{

	pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> ransac;
	ransac.setInputSource(source_cloud);		// Դ����
	ransac.setInputTarget(target_cloud);     // Ŀ�����

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_source(new pcl::PointCloud<pcl::FPFHSignature33>);
	compute_fpfh(source_cloud, fpfhs_source);
	ransac.setSourceFeatures(fpfhs_source);   // Դ����FPFH����

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_target(new pcl::PointCloud<pcl::FPFHSignature33>);
	compute_fpfh(target_cloud, fpfhs_target);
	ransac.setTargetFeatures(fpfhs_target);    // Ŀ�����FPFH����

	ransac.setCorrespondenceRandomness(5);      // ��ѡ�����������Ӧʱ������Ҫʹ�õ��ھӵ�����,��ֵԽ������ƥ��������Խ��
	ransac.setInlierFraction(0.5f);             // �����(�����)inlier����
	ransac.setNumberOfSamples(3);               // ÿ�ε�����ʹ�õĲ���������
	ransac.setSimilarityThreshold(0.1f);        // ���ײ����ζ�Ӧ�ܾ�������ı�Ե����֮���������ֵ����Ϊ[0,1]������1Ϊ��ȫƥ�䡣
	ransac.setMaxCorrespondenceDistance(1.0f);  // �ڵ㣬��ֵ Inlier threshold
	ransac.setMaximumIterations(1000);           // RANSAC ������������

	//������Ҫ�ĸ���任�Ա㽫����ĵ���ƥ�䵽Ŀ�����
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	ransac.align(*Final);

	tf = ransac.getFinalTransformation().cast<float>();
}


// ����׼

void ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf) {

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	//----------------------icp���Ĵ���--------------------
	icp.setInputSource(source);						// Դ����
	icp.setInputTarget(target);						// Ŀ�����
	icp.setTransformationEpsilon(1e-6);             // Ϊ��ֹ����������Сת������
	icp.setMaxCorrespondenceDistance(3);          // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
	icp.setEuclideanFitnessEpsilon(1e-6);           // �������������Ǿ�������С����ֵ�� ֹͣ������
	icp.setMaximumIterations(1250);                   // ����������
	icp.setRANSACOutlierRejectionThreshold(0.05);   // �޳�������ƣ����� RANSAC �㷨�����������
	//icp.setUseReciprocalCorrespondences(true);    // ����Ϊtrue,��ʹ���໥��Ӧ��ϵ

	// ������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	icp.align(*Final);

	tf = icp.getFinalTransformation().cast<float>();
}

void KD_ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf) {

	pcl::IterativeClosestPoint<PointT, PointT> icp;

	//---------------------KD����������--------------------
	pcl::search::KdTree<PointT>::Ptr source_tree(new pcl::search::KdTree<PointT>);
	source_tree->setInputCloud(source);
	pcl::search::KdTree<PointT>::Ptr target_tree(new pcl::search::KdTree<PointT>);
	target_tree->setInputCloud(target);
	icp.setSearchMethodSource(source_tree);
	icp.setSearchMethodTarget(target_tree);

	//----------------------icp���Ĵ���--------------------
	icp.setInputSource(source);						// Դ����
	icp.setInputTarget(target);						// Ŀ�����
	icp.setTransformationEpsilon(1e-10);             // Ϊ��ֹ����������Сת������
	icp.setMaxCorrespondenceDistance(10);          // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
	icp.setEuclideanFitnessEpsilon(1e-3);           // �������������Ǿ�������С����ֵ�� ֹͣ������
	icp.setMaximumIterations(250);                   // ����������
	icp.setRANSACOutlierRejectionThreshold(0.05);   // �޳�������ƣ����� RANSAC �㷨�����������
	//icp.setUseReciprocalCorrespondences(true);    // ����Ϊtrue,��ʹ���໥��Ӧ��ϵ

	// ������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	icp.align(*Final);

	tf = icp.getFinalTransformation().cast<float>();
}

void SGI_ICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f& tf) {

	pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;

	//---------------------KD����������--------------------
	pcl::search::KdTree<PointT>::Ptr source_tree(new pcl::search::KdTree<PointT>);
	source_tree->setInputCloud(source);
	pcl::search::KdTree<PointT>::Ptr target_tree(new pcl::search::KdTree<PointT>);
	target_tree->setInputCloud(target);
	gicp.setSearchMethodSource(source_tree);
	gicp.setSearchMethodTarget(target_tree);

	//-----------------����GICP��ز���-----------------------
	gicp.setInputSource(source);  //Դ����
	gicp.setInputTarget(target);  //Ŀ�����
	gicp.setMaxCorrespondenceDistance(1.0); //���ö�Ӧ���֮���������
	gicp.setTransformationEpsilon(1e-10);   //Ϊ��ֹ����������Сת������
	gicp.setEuclideanFitnessEpsilon(1e-3);  //�������������Ǿ�������С����ֵ�� ֹͣ����
	gicp.setMaximumIterations(35); //����������  
	//gicp.setUseReciprocalCorrespondences(true);//ʹ���໥��Ӧ��ϵ

	// ������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
	gicp.align(*Final);

	tf = gicp.getFinalTransformation().cast<float>();
}


void N_ICP(const pcl::PointCloud<PointNormalT>::Ptr source, const pcl::PointCloud<PointNormalT>::Ptr target, Eigen::Matrix4f& tf)
{

	pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT>n_icp;

	n_icp.setInputSource(source);
	n_icp.setInputTarget(target);
	n_icp.setTransformationEpsilon(1e-10);		// Ϊ��ֹ����������Сת������
	n_icp.setMaxCorrespondenceDistance(2);	// ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
	n_icp.setEuclideanFitnessEpsilon(1e-6);		// �������������Ǿ�������С����ֵ�� ֹͣ������
	n_icp.setUseReciprocalCorrespondences(false);// ����Ϊtrue���Ϊ��һ���㷨
	n_icp.setMaximumIterations(150);				// ����������

	// ������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<PointNormalT>::Ptr Final(new pcl::PointCloud<PointNormalT>);
	n_icp.align(*Final);

	tf = n_icp.getFinalTransformation().cast<float>();
}


// �㵽��ICP(��������С�����Ż�)
void lm_ICP(const pcl::PointCloud<PointNormalT>::Ptr source_with_normals, const pcl::PointCloud<PointNormalT>::Ptr target_with_normals,
	Eigen::Matrix4f& tf, float distance)
{
	pcl::IterativeClosestPoint<PointNormalT, PointNormalT>lm_icp;
	pcl::registration::TransformationEstimationPointToPlane<PointNormalT, PointNormalT>::Ptr PointToPlane
	(new pcl::registration::TransformationEstimationPointToPlane<PointNormalT, PointNormalT>);
	lm_icp.setTransformationEstimation(PointToPlane);
	lm_icp.setInputSource(source_with_normals);
	lm_icp.setInputTarget(target_with_normals);
	lm_icp.setTransformationEpsilon(1e-10);				// Ϊ��ֹ����������Сת������
	lm_icp.setMaxCorrespondenceDistance(distance);		// ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
	lm_icp.setEuclideanFitnessEpsilon(1e-4);			// �������������Ǿ�������С����ֵ�� ֹͣ������
	lm_icp.setMaximumIterations(500);					 // ����������
	lm_icp.setUseReciprocalCorrespondences(false);		//ʹ���໥��Ӧ��ϵ

	// ������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<PointNormalT>::Ptr Final(new pcl::PointCloud<PointNormalT>);
	lm_icp.align(*Final);

	tf = lm_icp.getFinalTransformation().cast<float>();

}

void pairAlign(const pcl::PointCloud<PointT>::Ptr cloud_src, const pcl::PointCloud<PointT>::Ptr cloud_tgt, Eigen::Matrix4f &final_transform, bool downsample)
{
	pcl::PointCloud<PointT>::Ptr src(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr tgt(new pcl::PointCloud<PointT>);

	// �Ƿ��ȶԵ��ƽ����²���
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

	//�������淨�ߺ�����
	pcl::PointCloud<PointNormalT>::Ptr points_with_normals_src(new pcl::PointCloud<PointNormalT>);
	cloud_with_normal(src, points_with_normals_src);

	pcl::PointCloud<PointNormalT>::Ptr points_with_normals_tgt(new pcl::PointCloud<PointNormalT>);
	cloud_with_normal(tgt, points_with_normals_tgt);

	// ������׼
	lm_ICP(points_with_normals_src, points_with_normals_tgt, final_transform);

}


// �㼯��׼
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
	// ������ʽδ������R������ֵҪȡ��
	if (determinant < 0) {
		R_ans = -U * V.transpose();
	}
	else {
		R_ans = U * V.transpose();
	}
	// �����������R����t
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