#pragma once
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/voxel_grid.h>						// 体素采样
#include <pcl/keypoints/uniform_sampling.h>				// 均匀采样
#include <pcl/filters/random_sample.h>					// 随机采样
#include <pcl/filters/normal_space.h>					// 法线空间采样
#include <pcl/filters/sampling_surface_normal.h>		// 索引空间采样

#include <pcl/filters/passthrough.h>					// 直通滤波
#include <pcl/filters/statistical_outlier_removal.h>	// 统计滤波
#include <pcl/filters/radius_outlier_removal.h>			// 半径滤波
#include <pcl/filters/conditional_removal.h>			// 条件滤波
#include <pcl/filters/extract_indices.h>				// 索引提取
#include <pcl/filters/project_inliers.h>				// 投影滤波
#include <pcl/filters/model_outlier_removal.h>			// 模型滤波
#include <pcl/filters/convolution_3d.h>					// 高斯滤波

#include "utils.h"


// -----------------体素采样-------------------
void downSampleVoxelGrids(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, float lx, float ly, float lz)
{
	/*
	根据输入的点云，首先计算一个能够刚好包裹住该点云的立方体
	然后根据设定的分辨率，将该大立方体分割成不同的小立方体
	对于每一个小立方体内的点，计算他们的质心，并用该质心的坐标来近似该立方体内的若干点
	*/
	pcl::VoxelGrid<PointT> vg;			// 创建滤波器对象
	vg.setInputCloud(cloud);			// 设置待滤波的点云
	vg.setLeafSize(lx, ly, lz);			// 设置最小体素边长 单位 m
	vg.filter(*cloud_filtered);			// 执行滤波，保存滤波结果于cloud_filtered

}

// -----------------均匀采样------------------- 
void uniformSampling(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, double radius)
{
	/*
	通过构建指定半径的球体对点云进行下采样滤波，
	将每一个球内距离球体中心最近的点作为下采样之后的点输出
	*/
	pcl::UniformSampling<PointT> us;
	us.setInputCloud(cloud);
	us.setRadiusSearch(radius);	//设置滤波是创建球的半径
	us.filter(*cloud_filtered);

}

// -----------------随机采样------------------- 
void randomSampling(const pcl::PointCloud<PointT>::Ptr cloud, int sample_num, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{

	pcl::RandomSample<PointT> rs;
	rs.setInputCloud(cloud);
	rs.setSample(sample_num);
	rs.filter(*cloud_filtered);

}

// -----------------法线空间采样------------------- 
void normalSpaceSampling(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
	/*
	在法向量空间内均匀随机采样，使所选点之间的法线分布尽可能大
	结果表现为地物特征变化大的地方剩余点较多，变化小的地方剩余点稀少，可有效保护地物特征
	*/
	pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss;
	nss.setInputCloud(cloud);// 设置输入点云
	nss.setNormals(normals);// 设置在输入点云上计算的法线
	nss.setBins(2, 2, 2);// 设置x,y,z方向bins的个数
	nss.setSeed(0); // 设置种子点
	nss.setSample(1000); // 设置要采样的索引数。
	nss.filter(*cloud_filtered);

}

// -----------------索引空间采样------------------- 
void surfaceNormalSampling(const pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals, pcl::PointCloud<PointNormalT>::Ptr& cloud_filtered)
{
	/*
	将输入空间划分为网格，直到每个网格中最多包含N个点，并在每个网格中随机采样
	使用每个网格的N个点来计算Normal,网格内所有点都被赋予相同的法线
	*/
	pcl::SamplingSurfaceNormal <pcl::PointNormal> ssn;
	ssn.setInputCloud(cloud_with_normals);
	ssn.setSample(10);     // 设置每个网格的最大样本数 n
	ssn.setRatio(0.1f);    // 设置采样率&
	ssn.filter(*cloud_filtered);// 输出的点云是用每个网格里的最大样本数n x &

}

// -----------------直通滤波-------------------
void pass_through_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, char *axis, int min, int max)
{
	/*
	指定一个维度以及该维度下的值域
	遍历点云中的每个点，判断该点在指定维度上的取值是否在值域内，删除取值不在值域内的点
	遍历结束，留下的点即构成滤波后的点云。
	*/
	pcl::PassThrough<PointT> pass;      //创建滤波器对象
	pass.setInputCloud(cloud);			//设置待滤波的点云
	pass.setFilterFieldName(axis);		//设置在Z轴方向上进行滤波
	pass.setFilterLimits(min, max);		//设置滤波范围为0~1,在范围之外的点会被剪除
	pass.filter(*cloud_filtered);		//开始过滤
}

// -----------------统计滤波-------------------
void statistical_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, int nr_k, double stddev_mult)
{
	/*
	统计滤波器主要用于去除明显离群点。 离群点特征在空间中分布稀疏。
	定义某处点云小于某个密度，既点云无效。
	计算每个点到其最近的k个点平均距离。则点云中所有点的距离应构成高斯分布。
	根据给定均值与方差，可剔除方差之外的点。
	*/
	pcl::StatisticalOutlierRemoval<PointT> sor;	// 创建滤波器对象
	sor.setInputCloud(cloud);                   // 设置待滤波的点云
	sor.setMeanK(nr_k);                         // 设置在进行统计时考虑查询点邻近点数
	sor.setStddevMulThresh(stddev_mult);        // 设置判断是否为离群点的阈值
	sor.filter(*cloud_filtered);                // 执行滤波处理保存内点到cloud_filtered

}

// -----------------半径滤波-------------------
void radius_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered, double radius, int min_pts)
{
	/*
	半径滤波器以某点为中心画一个圆计算落在该圆中点的数量，
	当数量大于给定值时，则保留该点，数量小于给定值则剔除该点。
	*/
	pcl::RadiusOutlierRemoval<PointT> ror;
	ror.setInputCloud(cloud);				// 输入点云
	ror.setRadiusSearch(radius);			// 设置半径为0.1m范围内找临近点
	ror.setMinNeighborsInRadius(min_pts);	// 设置查询点的邻域点集数小于10删除
	ror.filter(*cloud_filtered);			// 执行滤波
}

// -----------------条件滤波-------------------
void condition_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
	/*
	条件滤波器通过设定滤波条件进行滤波，删除不符合用户指定的一个或者多个条件。
	直通滤波器是一种较简单的条件滤波器。
	*/
	pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>());//创建条件定义对象range_cond
	//为条件定义对象添加比较算子
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
		pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, -0.1)));	//添加在x字段上大于 -0.1 的比较算子
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
		pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, 1.0)));	//添加在x字段上小于 1.0 的比较算子
	pcl::ConditionalRemoval<PointT> cr;	//创建滤波器对象
	cr.setCondition(range_cond);				//用条件定义对象初始化
	cr.setInputCloud(cloud);					//设置待滤波点云
	//cr.setKeepOrganized(true);				//设置保持点云的结构
	//cr.setUserFilterValue(5);					//将过滤掉的点用（5，5，5）代替
	cr.filter(*cloud_filtered);					//执行滤波，保存滤波结果于cloud_filtered
}


// -----------------索引提取-------------------
void extract_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
	pcl::ExtractIndices<PointT> extract;		//创建点云提取对象
	extract.setInputCloud(cloud);				//设置输入点云
	extract.setIndices(inliers);				//设置分割后的内点inliers为需要提取的点集
	extract.setNegative(false);					//设置提取内点而非外点，默认false
	extract.filter(*cloud_filtered);			//提取点集并存储到 cloud_filtered
}


// -----------------投影滤波-------------------
void project_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<PointT>::Ptr& cloud_projected)
{
	/*
	将点投影到一个参数化模型上，这个参数化模型可以是平面、圆球、圆柱、锥形等进行投影滤波。
	把三维点云投影到二维图像上，然后用图像处理的方法进行处理。
	*/
	pcl::ProjectInliers<pcl::PointXYZ> proj;//创建投影滤波器对象
	proj.setModelType(pcl::SACMODEL_PLANE);	//设置对象对应的投影模型
	proj.setInputCloud(cloud);				//设置输入点云
	proj.setModelCoefficients(coefficients);//设置模型对应的系数
	proj.filter(*cloud_projected);			//执行投影滤波，存储结果于cloud_projected
}

// -----------------模型滤波-------------------
void model_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients model_coeff, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
	/*
	根据点到模型的距离，设置距离阈值过滤非模型点, 基于模型的点分割操作，将模型外的点从点云中剔除。
	*/
	pcl::ModelOutlierRemoval<PointT> filter;		//创建模型滤波器对象
	filter.setModelCoefficients(model_coeff);		//为模型对象添加模型系数
	filter.setThreshold(0.1);						//设置判断是否为模型内点的阈值
	filter.setModelType(pcl::SACMODEL_PLANE);		//设置模型类别
	filter.setInputCloud(cloud);					//输入待滤波点云
	filter.setNegative(false);						//默认false，提取模型内点；true，提取模型外点
	filter.filter(*cloud_filtered);					//执行模型滤波，保存滤波结果于cloud_filtered

}

// -----------------高斯滤波-------------------
void gaussian_filter(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
	//设置kernel相关参数
	pcl::filters::GaussianKernel<PointT, PointT> kernel;
	kernel.setSigma(4);//高斯函数的标准方差，决定函数的宽度
	kernel.setThresholdRelativeToSigma(4);//设置相对Sigma参数的距离阈值
	kernel.setThreshold(0.05);//设置距离阈值，若点间距离大于阈值则不予考虑

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	//设置Convolution相关参数
	pcl::filters::Convolution3D<PointT, PointT, pcl::filters::GaussianKernel<PointT, PointT>> convolution;
	convolution.setKernel(kernel);//设置卷积核
	convolution.setInputCloud(cloud);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(0.01);
	convolution.convolve(*cloud_filtered);
}

// -----------------方框滤波-------------------
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

	pcl::copyPointCloud(*source_cloud, pi0, *extract_cloud);//将对应索引的点存储
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

	pcl::copyPointCloud(*source_cloud, pi0, *extract_cloud);//将对应索引的点存储
}

void remove_repeat_point(const pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointT>::Ptr& filtered_cloud)
{
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(source_cloud);

	std::vector<int> nn_indices;  // 存放近邻索引
	std::vector<float> nn_dists;  // 存放近邻距离

	float radius = 0.00001;		// 距离阈值，若两点之间的距离为0.000001则认为是重合点

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

	std::vector<int> pointIdxRadiusSearch;		// 保存每个近邻点的索引
	std::vector<float> PointRadiusSquaredDistance;	// 保存每个近邻点与查找点的欧氏距离的平方
	float radius = 0.01;			// 距离阈值,若两点间距离小于0.01，则认为是重合点
	std::vector<int> overlap_index; // 重合索引
	std::vector<int> remain_index;  // 剩余索引

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

	// 保留非重复索引
	pcl::copyPointCloud(*cloud_a, remain_index, *cloud_a_out);//将对应索引的点存储
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
