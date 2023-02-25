#pragma once
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/voxel_grid.h>						// ���ز���
#include <pcl/keypoints/uniform_sampling.h>				// ���Ȳ���
#include <pcl/filters/random_sample.h>					// �������
#include <pcl/filters/normal_space.h>					// ���߿ռ����
#include <pcl/filters/sampling_surface_normal.h>		// �����ռ����

#include <pcl/filters/passthrough.h>					// ֱͨ�˲�
#include <pcl/filters/statistical_outlier_removal.h>	// ͳ���˲�
#include <pcl/filters/radius_outlier_removal.h>			// �뾶�˲�
#include <pcl/filters/conditional_removal.h>			// �����˲�
#include <pcl/filters/extract_indices.h>				// ������ȡ
#include <pcl/filters/project_inliers.h>				// ͶӰ�˲�
#include <pcl/filters/model_outlier_removal.h>			// ģ���˲�
#include <pcl/filters/convolution_3d.h>					// ��˹�˲�

#include "utils.h"


// ���ز��� ���ƶ�����λ�ã�
void downSampleVoxelGrids(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, float lx = 3.0, float ly = 3.0, float lz = 3.0);

// ���Ȳ��� �����ƶ�����λ�ã�
void uniformSampling(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, double radius = 3.0);

// �������
void randomSampling(const pcl::PointCloud<PointT>::Ptr cloud, int sample_num, pcl::PointCloud<PointT>::Ptr& cloud_filtered);

// ���߿ռ����
void normalSpaceSampling(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<PointT>::Ptr& cloud_filtered);

// �����ռ����
void surfaceNormalSampling(const pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals, pcl::PointCloud<PointNormalT>::Ptr& cloud_filtered);

// ֱͨ�˲�
void pass_through_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, char *axis = "z", int min = -300, int max = 300);

// ͳ���˲�
void statistical_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, int nr_k = 50, double stddev_mult = 2.0);

// �뾶�˲�
void radius_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, double radius = 3.0, int min_pts = 10);

void condition_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered);

void extract_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::PointCloud<PointT>::Ptr& cloud_filtered);

void project_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<PointT>::Ptr& cloud_projected);

void model_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients model_coeff, pcl::PointCloud<PointT>::Ptr& cloud_filtered);

void gaussian_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered);

// �����˲�
void box_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, Eigen::Vector4f min_pt, Eigen::Vector4f max_pt);
// �����˲�
void kdtree_Filter(const pcl::PointCloud<PointT>::Ptr & pcloud, const PointT & pnt, double r, pcl::PointCloud<PointT>::Ptr & out_cloud);


void removeZeoPoints(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointT>::Ptr& extract_cloud);
void remove_tf_zeo_point(const pcl::PointCloud<PointT>::Ptr source_cloud, const Eigen::Matrix4f T_c2b, pcl::PointCloud<PointT>::Ptr &extract_cloud);

// ɾ�������ڲ��غϵ�
void remove_repeat_point(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointT>::Ptr& filtered_cloud);

// ɾ���������Ƶ��غϵ�
void remove_overlap_cloud(const pcl::PointCloud<PointT>::Ptr cloud_a, const pcl::PointCloud<PointT>::Ptr cloud_b, pcl::PointCloud<PointT>::Ptr &cloud_a_out);


// -----------------���ز���-------------------
void downSampleVoxelGrids(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, float lx, float ly, float lz)
{
	/*
	��������ĵ��ƣ����ȼ���һ���ܹ��պð���ס�õ��Ƶ�������
	Ȼ������趨�ķֱ��ʣ����ô�������ָ�ɲ�ͬ��С������
	����ÿһ��С�������ڵĵ㣬�������ǵ����ģ����ø����ĵ����������Ƹ��������ڵ����ɵ�
	*/
	pcl::VoxelGrid<PointT> vg;			// �����˲�������
	vg.setInputCloud(cloud);			// ���ô��˲��ĵ���
	vg.setLeafSize(lx, ly, lz);			// ������С���ر߳� ��λ m
	vg.filter(*cloud_filtered);			// ִ���˲��������˲������cloud_filtered

}

// -----------------���Ȳ���------------------- 
void uniformSampling(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, double radius)
{
	/*
	ͨ������ָ���뾶������Ե��ƽ����²����˲���
	��ÿһ�����ھ���������������ĵ���Ϊ�²���֮��ĵ����
	*/
	pcl::UniformSampling<PointT> us;
	us.setInputCloud(cloud);
	us.setRadiusSearch(radius);	//�����˲��Ǵ�����İ뾶
	us.filter(*cloud_filtered);

}

// -----------------�������------------------- 
void randomSampling(const pcl::PointCloud<PointT>::Ptr cloud, int sample_num, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{

	pcl::RandomSample<PointT> rs;
	rs.setInputCloud(cloud);
	rs.setSample(sample_num);
	rs.filter(*cloud_filtered);

}

// -----------------���߿ռ����------------------- 
void normalSpaceSampling(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
	/*
	�ڷ������ռ��ھ������������ʹ��ѡ��֮��ķ��߷ֲ������ܴ�
	�������Ϊ���������仯��ĵط�ʣ���϶࣬�仯С�ĵط�ʣ���ϡ�٣�����Ч������������
	*/
	pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss;
	nss.setInputCloud(cloud);// �����������
	nss.setNormals(normals);// ��������������ϼ���ķ���
	nss.setBins(2, 2, 2);// ����x,y,z����bins�ĸ���
	nss.setSeed(0); // �������ӵ�
	nss.setSample(1000); // ����Ҫ��������������
	nss.filter(*cloud_filtered);

}

// -----------------�����ռ����------------------- 
void surfaceNormalSampling(const pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals, pcl::PointCloud<PointNormalT>::Ptr& cloud_filtered)
{
	/*
	������ռ仮��Ϊ����ֱ��ÿ��������������N���㣬����ÿ���������������
	ʹ��ÿ�������N����������Normal,���������е㶼��������ͬ�ķ���
	*/
	pcl::SamplingSurfaceNormal <pcl::PointNormal> ssn;
	ssn.setInputCloud(cloud_with_normals);
	ssn.setSample(10);     // ����ÿ���������������� n
	ssn.setRatio(0.1f);    // ���ò�����&
	ssn.filter(*cloud_filtered);// ����ĵ�������ÿ������������������n x &

}

// -----------------ֱͨ�˲�-------------------
void pass_through_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, char *axis, int min, int max)
{
	/*
	ָ��һ��ά���Լ���ά���µ�ֵ��
	���������е�ÿ���㣬�жϸõ���ָ��ά���ϵ�ȡֵ�Ƿ���ֵ���ڣ�ɾ��ȡֵ����ֵ���ڵĵ�
	�������������µĵ㼴�����˲���ĵ��ơ�
	*/
	pcl::PassThrough<PointT> pass;      //�����˲�������
	pass.setInputCloud(cloud);			//���ô��˲��ĵ���
	pass.setFilterFieldName(axis);		//������Z�᷽���Ͻ����˲�
	pass.setFilterLimits(min, max);		//�����˲���ΧΪ0~1,�ڷ�Χ֮��ĵ�ᱻ����
	pass.filter(*cloud_filtered);		//��ʼ����
}

// -----------------ͳ���˲�-------------------
void statistical_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, int nr_k, double stddev_mult)
{
	/*
	ͳ���˲�����Ҫ����ȥ��������Ⱥ�㡣 ��Ⱥ�������ڿռ��зֲ�ϡ�衣
	����ĳ������С��ĳ���ܶȣ��ȵ�����Ч��
	����ÿ���㵽�������k����ƽ�����롣����������е�ľ���Ӧ���ɸ�˹�ֲ���
	���ݸ�����ֵ�뷽����޳�����֮��ĵ㡣
	*/
	pcl::StatisticalOutlierRemoval<PointT> sor;	// �����˲�������
	sor.setInputCloud(cloud);                   // ���ô��˲��ĵ���
	sor.setMeanK(nr_k);                         // �����ڽ���ͳ��ʱ���ǲ�ѯ���ڽ�����
	sor.setStddevMulThresh(stddev_mult);        // �����ж��Ƿ�Ϊ��Ⱥ�����ֵ
	sor.filter(*cloud_filtered);                // ִ���˲��������ڵ㵽cloud_filtered

}

// -----------------�뾶�˲�-------------------
void radius_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, double radius, int min_pts)
{
	/*
	�뾶�˲�����ĳ��Ϊ���Ļ�һ��Բ�������ڸ�Բ�е��������
	���������ڸ���ֵʱ�������õ㣬����С�ڸ���ֵ���޳��õ㡣
	*/
	pcl::RadiusOutlierRemoval<PointT> ror;
	ror.setInputCloud(cloud);				// �������
	ror.setRadiusSearch(radius);			// ���ð뾶Ϊ0.1m��Χ�����ٽ���
	ror.setMinNeighborsInRadius(min_pts);	// ���ò�ѯ�������㼯��С��10ɾ��
	ror.filter(*cloud_filtered);			// ִ���˲�
}

// -----------------�����˲�-------------------
void condition_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
	/*
	�����˲���ͨ���趨�˲����������˲���ɾ���������û�ָ����һ�����߶��������
	ֱͨ�˲�����һ�ֽϼ򵥵������˲�����
	*/
	pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>());//���������������range_cond
	//Ϊ�������������ӱȽ�����
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
		pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, -0.1)));	//�����x�ֶ��ϴ��� -0.1 �ıȽ�����
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
		pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, 1.0)));	//�����x�ֶ���С�� 1.0 �ıȽ�����
	pcl::ConditionalRemoval<PointT> cr;	//�����˲�������
	cr.setCondition(range_cond);				//��������������ʼ��
	cr.setInputCloud(cloud);					//���ô��˲�����
	//cr.setKeepOrganized(true);				//���ñ��ֵ��ƵĽṹ
	//cr.setUserFilterValue(5);					//�����˵��ĵ��ã�5��5��5������
	cr.filter(*cloud_filtered);					//ִ���˲��������˲������cloud_filtered
}


// -----------------������ȡ-------------------
void extract_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
	pcl::ExtractIndices<PointT> extract;		//����������ȡ����
	extract.setInputCloud(cloud);				//�����������
	extract.setIndices(inliers);				//���÷ָ����ڵ�inliersΪ��Ҫ��ȡ�ĵ㼯
	extract.setNegative(false);					//������ȡ�ڵ������㣬Ĭ��false
	extract.filter(*cloud_filtered);			//��ȡ�㼯���洢�� cloud_filtered
}


// -----------------ͶӰ�˲�-------------------
void project_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<PointT>::Ptr& cloud_projected)
{
	/*
	����ͶӰ��һ��������ģ���ϣ����������ģ�Ϳ�����ƽ�桢Բ��Բ����׶�εȽ���ͶӰ�˲���
	����ά����ͶӰ����άͼ���ϣ�Ȼ����ͼ����ķ������д���
	*/
	pcl::ProjectInliers<pcl::PointXYZ> proj;//����ͶӰ�˲�������
	proj.setModelType(pcl::SACMODEL_PLANE);	//���ö����Ӧ��ͶӰģ��
	proj.setInputCloud(cloud);				//�����������
	proj.setModelCoefficients(coefficients);//����ģ�Ͷ�Ӧ��ϵ��
	proj.filter(*cloud_projected);			//ִ��ͶӰ�˲����洢�����cloud_projected
}

// -----------------ģ���˲�-------------------
void model_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients model_coeff, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
	/*
	���ݵ㵽ģ�͵ľ��룬���þ�����ֵ���˷�ģ�͵�, ����ģ�͵ĵ�ָ��������ģ����ĵ�ӵ������޳���
	*/
	pcl::ModelOutlierRemoval<PointT> filter;		//����ģ���˲�������
	filter.setModelCoefficients(model_coeff);		//Ϊģ�Ͷ������ģ��ϵ��
	filter.setThreshold(0.1);						//�����ж��Ƿ�Ϊģ���ڵ����ֵ
	filter.setModelType(pcl::SACMODEL_PLANE);		//����ģ�����
	filter.setInputCloud(cloud);					//������˲�����
	filter.setNegative(false);						//Ĭ��false����ȡģ���ڵ㣻true����ȡģ�����
	filter.filter(*cloud_filtered);					//ִ��ģ���˲��������˲������cloud_filtered

}

// -----------------��˹�˲�-------------------
void gaussian_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
	//����kernel��ز���
	pcl::filters::GaussianKernel<PointT, PointT> kernel;
	kernel.setSigma(4);//��˹�����ı�׼������������Ŀ��
	kernel.setThresholdRelativeToSigma(4);//�������Sigma�����ľ�����ֵ
	kernel.setThreshold(0.05);//���þ�����ֵ���������������ֵ���迼��

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	//����Convolution��ز���
	pcl::filters::Convolution3D<PointT, PointT, pcl::filters::GaussianKernel<PointT, PointT>> convolution;
	convolution.setKernel(kernel);//���þ����
	convolution.setInputCloud(cloud);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(0.01);
	convolution.convolve(*cloud_filtered);
}

// -----------------�����˲�-------------------
void box_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, Eigen::Vector4f min_pt, Eigen::Vector4f max_pt)
{
	std::vector<int> indices;
	pcl::getPointsInBox(*cloud, min_pt, max_pt, indices);
	pcl::copyPointCloud(*cloud, indices, *cloud_filtered);
}


void kdtree_Filter(const pcl::PointCloud<PointT>::Ptr & pcloud, const PointT & pnt, double r, pcl::PointCloud<PointT>::Ptr & out_cloud)
{
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(pcloud);
	std::vector<int> pointId;
	std::vector<float> pointSquareDistance;
	kdtree.radiusSearch(pnt, r, pointId, pointSquareDistance);
	int num = pointId.size();
	pcl::PointCloud<PointT>::Ptr my_cloud(new pcl::PointCloud<PointT>);
	my_cloud->width = num;
	my_cloud->height = 1;
	my_cloud->is_dense = false;
	my_cloud->points.resize(my_cloud->width * my_cloud->height);
	for (size_t i = 0; i < num; i++)
	{
		//my_cloud->push_back(pcloud->points[i]);
		my_cloud->points[i] = pcloud->points[pointId[i]];
	}

	out_cloud = my_cloud;
}

void removeZeoPoints(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointT>::Ptr& extract_cloud)
{

	pcl::PointIndices pi0;
	for (int i = 0; i < source_cloud->points.size(); ++i)
	{
		if (source_cloud->points[i].x != 0
			&& source_cloud->points[i].y != 0
			&& source_cloud->points[i].z != 0)
		{
			pi0.indices.push_back(i);
		}
	}

	pcl::copyPointCloud(*source_cloud, pi0, *extract_cloud);//����Ӧ�����ĵ�洢
}

void remove_tf_zeo_point(const pcl::PointCloud<PointT>::Ptr source_cloud, const Eigen::Matrix4f T_c2b, pcl::PointCloud<PointT>::Ptr &extract_cloud)
{
	PointT point_valid(0, 0, 0);

	pcl::PointCloud<PointT>::Ptr zeo_cloud(new pcl::PointCloud<PointT>);
	zeo_cloud->push_back(point_valid);

	pcl::PointCloud<PointT>::Ptr tf_zeo_cloud(new pcl::PointCloud<PointT>);
	pcl::transformPointCloud(*zeo_cloud, *tf_zeo_cloud, T_c2b);

	pcl::PointIndices pi0;
	for (int i = 0; i < source_cloud->points.size(); ++i)
	{
		if (source_cloud->points[i].x != tf_zeo_cloud->points[0].x
			&& source_cloud->points[i].y != tf_zeo_cloud->points[0].y
			&& source_cloud->points[i].z != tf_zeo_cloud->points[0].z)
		{
			pi0.indices.push_back(i);
		}
	}

	pcl::copyPointCloud(*source_cloud, pi0, *extract_cloud);//����Ӧ�����ĵ�洢
}

void remove_repeat_point(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointT>::Ptr& filtered_cloud)
{
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(source_cloud);

	std::vector<int> nn_indices;  // ��Ž�������
	std::vector<float> nn_dists;  // ��Ž��ھ���

	float radius = 0.00001;		// ������ֵ��������֮��ľ���Ϊ0.000001����Ϊ���غϵ�

	pcl::PointIndices::Ptr outIdex(new pcl::PointIndices);
	for (int i = 0; i < source_cloud->points.size(); ++i)
	{
		PointT curPoint = source_cloud->points[i];
		if (tree->radiusSearch(curPoint, radius, nn_indices, nn_dists) > 0)
		{
			if (nn_indices.size() != 1)
			{
				for (size_t i = 0; i < nn_indices.size(); i++)
				{
					outIdex->indices.push_back(nn_indices[i]);
				}
			}
		}

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(source_cloud);
		extract.setIndices(outIdex);
		extract.setNegative(true);
		extract.filter(*filtered_cloud);
	}

}

void remove_overlap_cloud(const pcl::PointCloud<PointT>::Ptr cloud_a, const pcl::PointCloud<PointT>::Ptr cloud_b, pcl::PointCloud<PointT>::Ptr &cloud_a_out) {

	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud(cloud_b);

	std::vector<int> pointIdxRadiusSearch;		// ����ÿ�����ڵ������
	std::vector<float> PointRadiusSquaredDistance;	// ����ÿ�����ڵ�����ҵ��ŷ�Ͼ����ƽ��
	float radius = 0.01;			// ������ֵ,����������С��0.01������Ϊ���غϵ�
	std::vector<int> overlap_index; // �غ�����
	std::vector<int> remain_index;  // ʣ������

	for (size_t i = 0; i < cloud_a->points.size(); i++)
	{
		PointT searchPoint = cloud_a->points[i];

		if (tree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, PointRadiusSquaredDistance) > 0)
		{
			if (pointIdxRadiusSearch.size() != 1)
			{
				for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
				{
					overlap_index.push_back(pointIdxRadiusSearch[j]);
				}
			}
		}
		else
		{
			remain_index.push_back(i);
		}

	}

	// �������ظ�����
	pcl::copyPointCloud(*cloud_a, remain_index, *cloud_a_out);//����Ӧ�����ĵ�洢
}


void getTiming(std::string note) {
	static std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
	auto time_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic);
	tic = toc;
	if (!note.empty())
		std::cout << "[TIMING] " << note << ": " << time_elapsed.count() / 1000000.0 << " ms\n";
}

template <typename PointT>
void addNoiseToPointCloud(pcl::PointCloud<PointT>& cloud, float noise_std) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<> dis(0, noise_std);
	for (int idx = 0; idx < cloud.points.size(); ++idx) {
		cloud[idx].x = cloud[idx].x + static_cast<float> (dis(gen));
		cloud[idx].y = cloud[idx].y + static_cast<float> (dis(gen));
		cloud[idx].z = cloud[idx].z + static_cast<float> (dis(gen));
	}
}

template <typename PointT>
int intersectPointCloud(const typename pcl::PointCloud<PointT>::Ptr& source,
	const typename pcl::PointCloud<PointT>::Ptr& target,
	float epsilon) {
	typename pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>);
	tree->setInputCloud(target);
	std::vector<int> index(1);
	std::vector<float> sqr_distance(1);
	int count = 0;
	float sqr_epsilon = epsilon * epsilon;
	for (int idx = 0; idx < source->points.size(); ++idx) {
		tree->nearestKSearch(source->points[idx], 1, index, sqr_distance);
		if (sqr_distance[0] < sqr_epsilon)
			count++;
	}
	return count;
}