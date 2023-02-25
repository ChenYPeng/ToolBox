#pragma once
#include <vector>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/ransac.h> // RANSAC
#include <pcl/sample_consensus/sac_model_line.h> // 拟合3D直线
#include <pcl/sample_consensus/sac_model_circle3D.h> // 拟合3D圆

#include "utils.h"

// 判断投影方向
void determining_projection_types(const pcl::PointCloud<PointT>::Ptr source_cloud, PointT &Pmin, PointT &Pmax, int &fit_type);

// 空间直线拟合
void fit_2d_line(const std::vector<Point2D>& plane_cloud, Eigen::VectorXd &coeffs);
void plot_2d_line(const Eigen::VectorXd coeffs, float x, float &y);
void fit_3d_line_ols(const pcl::PointCloud<PointT>::Ptr in_cloud, std::vector<Eigen::VectorXd>& coffes);
void fit_3d_line_ransac(const pcl::PointCloud<PointT>::Ptr cloud, Eigen::VectorXf& coeffs);
void plot_3d_line(const pcl::PointCloud<PointT>::Ptr in_cloud, const Eigen::VectorXf coeffs, pcl::PointCloud<PointT>::Ptr& out_cloud);

// 空间曲线拟合
void fit_2d_curve(const std::vector<Point2D>& plane_cloud, int K, Eigen::VectorXd & coffes);
void plot_2d_curve(const Eigen::VectorXd coeffs, float x, float &y);
void fit_3d_curve_ols(const pcl::PointCloud<PointT>::Ptr in_cloud, std::vector<Eigen::VectorXd>& coffes);
void plot_3d_curve(const pcl::PointCloud<PointT>::Ptr in_cloud, std::vector<Eigen::VectorXd> coffes, pcl::PointCloud<PointT>::Ptr& fit_space);

// 空间圆弧拟合
void fit_3d_circle_ols(const Eigen::MatrixXd M, Eigen::VectorXd& coeffs);
void fit_3d_circle_ransac(const pcl::PointCloud<PointT>::Ptr cloud, Eigen::VectorXf& coeffs);
void plot_3d_circle(const pcl::PointCloud<PointT>::Ptr in_cloud, const Eigen::VectorXf coeffs, pcl::PointCloud<PointT>::Ptr& out_cloud);

void fit_space(const pcl::PointCloud<PointT>::Ptr source_cloud, const int class_type, pcl::PointCloud<PointT>::Ptr& fit_space);

// 直线的起点、终点
void get_line_two_end(const pcl::PointCloud<PointT>::Ptr line, pcl::PointCloud<PointT>::Ptr &line_out);
// 圆弧的起点、中点 、终点
void get_cricle_three_end(const pcl::PointCloud<PointT>::Ptr circle, const Eigen::VectorXf coffes, pcl::PointCloud<PointT>::Ptr &cricle_out);
// 曲线按步长输出各点
void get_curve_step_point(const pcl::PointCloud<PointT>::Ptr curve, pcl::PointCloud<PointT>::Ptr &curve_out);

// 根据检测的类别输出对应的端点
void get_weld_end_point(const std::vector<ExtractResult> ext_result, ExtractResultL & ext_result_list);


// 判断投影类型
void determining_projection_types(
	const pcl::PointCloud<PointT>::Ptr source_cloud,
	PointT &Pmin, PointT &Pmax, int &fit_type) {
	pcl::getMinMax3D(*source_cloud, Pmin, Pmax);

	float x = abs(Pmax.x - Pmin.x);
	float y = abs(Pmax.y - Pmin.y);
	float z = abs(Pmax.z - Pmin.z);

	float max = (x > y ? x : y) > z ? (x > y ? x : y) : z;
	//float min = (x < y ? x : y) < z ? (x < y ? x : y) : z;

	if (max == x)
	{
		fit_type = 1;  // XOY XOZ
	}
	else if (max == y)
	{
		fit_type = 2;  // YOX YOZ

	}
	else if (max == z)
	{
		fit_type = 3;  // ZOX ZOY

	}
	else
	{
		std::cout << "fit_type is wrong" << std::endl;
	}
}

void fit_2d_line(const std::vector<Point2D>& plane_cloud, Eigen::VectorXd &coeffs)
{
	double sum_x = 0.;
	double sum_y = 0.;
	double sum_xy = 0.;
	double sum_xx = 0.;

	int n = plane_cloud.size();

	for (auto point : plane_cloud)
	{
		double x = point.x;
		double y = point.y;

		sum_x += x;
		sum_y += y;
		sum_xx += x * x;
		sum_xy += x * y;
	}

	double k = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
	double b = (sum_y - k * sum_x) / n;

	coeffs = Eigen::VectorXd(2);
	coeffs << k, b;

}

void plot_2d_line(const Eigen::VectorXd coeffs, float x, float &y)
{
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); ++i)
	{
		y += coeffs.reverse()[i] * pow(x, i); //对 y=kx+b
	}

}

void fit_3d_line_ols(
	const pcl::PointCloud<PointT>::Ptr in_cloud,
	std::vector<Eigen::VectorXd>& coffes) {
	int fit_type;
	PointT Pmin, Pmax;
	determining_projection_types(in_cloud, Pmin, Pmax, fit_type);

	int N = in_cloud->points.size();
	std::vector<Point2D> cloud1(N);
	std::vector<Point2D> cloud2(N);

	switch (fit_type)
	{
	case 1:
	{
		//先投影到XOY平面, 在投影到XOZ平面
		for (size_t pId = 0; pId < N; pId++)
		{
			// 提取xy
			cloud1[pId].x = in_cloud->points[pId].x;
			cloud1[pId].y = in_cloud->points[pId].y;

			// 提取xz
			cloud2[pId].x = in_cloud->points[pId].x;
			cloud2[pId].y = in_cloud->points[pId].z;
		}
		break;
	}
	case 2:
	{
		// 分别在 XOY、XOZ 平面内拟合平面曲线
		for (size_t pId = 0; pId < N; pId++)
		{
			// 提取yx
			cloud1[pId].x = in_cloud->points[pId].y;
			cloud1[pId].y = in_cloud->points[pId].x;

			// 提取yz
			cloud2[pId].x = in_cloud->points[pId].y;
			cloud2[pId].y = in_cloud->points[pId].z;
		}
		break;
	}
	case 3:
	{
		for (size_t pId = 0; pId < N; pId++)
		{
			// 提取zx
			cloud1[pId].x = in_cloud->points[pId].z;
			cloud1[pId].y = in_cloud->points[pId].x;

			// 提取zy
			cloud2[pId].x = in_cloud->points[pId].z;
			cloud2[pId].y = in_cloud->points[pId].y;
		}
		break;
	}
	default:
		std::cout << "fit_type is wrong" << std::endl;
		break;
	}

	Eigen::VectorXd coeffs1, coeffs2;
	fit_2d_line(cloud1, coeffs1);
	fit_2d_line(cloud2, coeffs2);

	coffes.push_back(coeffs1);
	coffes.push_back(coeffs2);
}

void fit_3d_line_ransac(const pcl::PointCloud<PointT>::Ptr cloud, Eigen::VectorXf& coeffs)
{
	//----------------------RANSAC框架----------------------------   
	pcl::SampleConsensusModelLine<PointT>::Ptr model_line(new pcl::SampleConsensusModelLine<PointT>(cloud));
	pcl::RandomSampleConsensus<PointT> ransac(model_line);	//创建随机采样一致性对象
	ransac.setDistanceThreshold(0.5);	//内点到模型的最大距离
	ransac.setMaxIterations(300);		//最大迭代次数
	ransac.computeModel();				//执行RANSAC空间直线拟合
	ransac.getModelCoefficients(coeffs);
}

void plot_3d_line(
	const pcl::PointCloud<PointT>::Ptr in_cloud,
	const Eigen::VectorXf coeffs,
	pcl::PointCloud<PointT>::Ptr& out_cloud) {
	int fit_type;
	PointT Pmin, Pmax;
	determining_projection_types(in_cloud, Pmin, Pmax, fit_type);

	PointT Pts;
	float step = 1.0;
	float start, end;
	int num;

	float x1 = coeffs[0], y1 = coeffs[1], z1 = coeffs[2];
	float nx = coeffs[3], ny = coeffs[4], nz = coeffs[5];

	switch (fit_type)
	{
	case 1:
	{
		start = Pmin.x;
		end = Pmax.x;
		num = (end - start) / step;
		for (int pId = 0; pId < num; pId++)
		{
			Pts.x = start + step * pId;
			Pts.y = ny * (Pts.x - x1) / nx + y1;
			Pts.z = nz * (Pts.x - x1) / nx + z1;

			out_cloud->push_back(Pts);

		}
		break;
	}
	case 2:
	{
		start = Pmin.y;
		end = Pmax.y;
		num = (end - start) / step;

		for (int pId = 0; pId < num; pId++)
		{

			Pts.y = start + step * pId;
			Pts.x = nx * (Pts.y - y1) / ny + x1;
			Pts.z = nz * (Pts.y - y1) / ny + z1;

			out_cloud->push_back(Pts);
		}
		break;
	}
	case 3:
	{
		start = Pmin.z;
		end = Pmax.z;
		num = (end - start) / step;

		for (int pId = 0; pId < num; pId++)
		{
			Pts.z = start + step * pId;
			Pts.x = nx * (Pts.z - z1) / nz + x1;
			Pts.y = ny * (Pts.z - z1) / nz + y1;


			out_cloud->push_back(Pts);
		}
		break;
	}
	default:
		std::cout << "fit_type is wrong" << std::endl;
		break;
	}

	//剔除超出aabb包围框以外的点云
	int N = out_cloud->points.size();

	for (int pId = 0; pId < N; ++pId)
	{
		PointT Pi = out_cloud->points[pId];

		if ((Pmin.x <= Pi.x) && (Pi.x <= Pmax.x) &&
			(Pmin.y <= Pi.y) && (Pi.y <= Pmax.y) &&
			(Pmin.z <= Pi.z) && (Pi.z <= Pmax.z))
		{
			out_cloud->push_back(Pi);
		}
	}
}


void fit_2d_curve(const std::vector<Point2D>& plane_cloud, int K, Eigen::VectorXd & coffes)
{
	int N = plane_cloud.size(); // 行

	// 求解Ax=B

	// 创建系数矩阵A
	Eigen::MatrixXd A = Eigen::MatrixXd::Ones(N, K + 1);
	// 创建系数矩阵B
	Eigen::VectorXd B(N);

	// 遍历所有点构建系数矩阵
	for (int i = 0; i < N; ++i)
	{
		B(i) = plane_cloud[i].y;
		for (int k = K, dex = 0; k >= 1; --k, ++dex) // 遍历N到1阶
		{
			A(i, dex) = pow(plane_cloud[i].x, k);
		}
	}

	// 最小二乘解W
	coffes = Eigen::VectorXd(K);
	//W = (A.transpose() * A).inverse() * A.transpose() * B; // 间接平差法求解
	coffes = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B); //SVD求最小二乘解
	//std::cout << "拟合误差: " << (A * W - B).norm() / B.norm() << std::endl;

}

void plot_2d_curve(const Eigen::VectorXd coeffs, float x, float &y)
{
	for (int i = 0; i < coeffs.size(); ++i)
	{
		y += coeffs.reverse()[i] * pow(x, i); //对coeffs进行转置
	}
}

void fit_3d_curve_ols(
	const pcl::PointCloud<PointT>::Ptr in_cloud,
	std::vector<Eigen::VectorXd>& coffes) {
	int fit_type;
	PointT Pmin, Pmax;
	determining_projection_types(in_cloud, Pmin, Pmax, fit_type);

	int N = in_cloud->points.size();
	std::vector<Point2D> cloud1(N);
	std::vector<Point2D> cloud2(N);

	switch (fit_type)
	{
	case 1:
	{
		//先投影到XOY平面, 在投影到XOZ平面
		for (size_t pId = 0; pId < N; pId++)
		{
			// 提取xy
			cloud1[pId].x = in_cloud->points[pId].x;
			cloud1[pId].y = in_cloud->points[pId].y;

			// 提取xz
			cloud2[pId].x = in_cloud->points[pId].x;
			cloud2[pId].y = in_cloud->points[pId].z;
		}
		break;
	}
	case 2:
	{
		// 分别在 XOY、XOZ 平面内拟合平面曲线
		for (size_t pId = 0; pId < N; pId++)
		{
			// 提取yx
			cloud1[pId].x = in_cloud->points[pId].y;
			cloud1[pId].y = in_cloud->points[pId].x;

			// 提取yz
			cloud2[pId].x = in_cloud->points[pId].y;
			cloud2[pId].y = in_cloud->points[pId].z;
		}
		break;
	}
	case 3:
	{
		for (size_t pId = 0; pId < N; pId++)
		{
			// 提取zx
			cloud1[pId].x = in_cloud->points[pId].z;
			cloud1[pId].y = in_cloud->points[pId].x;

			// 提取zy
			cloud2[pId].x = in_cloud->points[pId].z;
			cloud2[pId].y = in_cloud->points[pId].y;
		}
		break;
	}
	default:
		std::cout << "fit_type is wrong" << std::endl;
		break;
	}

	Eigen::VectorXd coeffs1, coeffs2;
	fit_2d_curve(cloud1, 2, coeffs1);
	fit_2d_curve(cloud2, 6, coeffs2);

	coffes.push_back(coeffs1);
	coffes.push_back(coeffs2);
}

void plot_3d_curve(
	const pcl::PointCloud<PointT>::Ptr in_cloud,
	std::vector<Eigen::VectorXd> coffes,
	pcl::PointCloud<PointT>::Ptr& fit_space) {

	Eigen::VectorXd coffes1 = coffes[0];
	Eigen::VectorXd coffes2 = coffes[1];

	int fit_type;
	PointT Pmin, Pmax;
	determining_projection_types(in_cloud, Pmin, Pmax, fit_type);

	PointT Pts;
	float step = 2.0;
	float start, end;
	int num;

	switch (fit_type)
	{
	case 1:
	{
		start = Pmin.x;
		end = Pmax.x;
		num = (end - start) / step;

		for (int pId = 0; pId < num; ++pId)
		{
			Pts.x = start + step * pId;

			plot_2d_curve(coffes1, Pts.x, Pts.y);
			plot_2d_curve(coffes2, Pts.x, Pts.z);

			fit_space->push_back(Pts);
		}
		break;
	}
	case 2:
	{
		start = Pmin.y;
		end = Pmax.y;
		num = (end - start) / step;

		for (int pId = 0; pId < num; ++pId)
		{

			Pts.y = start + step * pId;

			plot_2d_curve(coffes1, Pts.y, Pts.x);
			plot_2d_curve(coffes2, Pts.y, Pts.z);

			fit_space->push_back(Pts);
		}
		break;
	}
	case 3:
	{
		start = Pmin.z;
		end = Pmax.z;
		num = (end - start) / step;

		for (int pId = 0; pId < num; ++pId)
		{

			Pts.z = start + step * pId;

			plot_2d_curve(coffes1, Pts.z, Pts.x);
			plot_2d_curve(coffes2, Pts.z, Pts.y);

			fit_space->push_back(Pts);
		}
		break;
	}
	default:
	{
		std::cout << "fit_type is wrong" << std::endl;
		break;
	}

	}

}

// OLS拟合三维空间圆弧
void fit_3d_circle_ols(const Eigen::MatrixXd M, Eigen::VectorXd& coeffs)
{
	int num = M.rows();
	Eigen::MatrixXd L1 = Eigen::MatrixXd::Ones(num, 1);
	Eigen::Vector3d A = (M.transpose() * M).inverse() * M.transpose() * L1;

	// 平面法向量
	auto A1 = A;
	A1.normalize();

	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num - 1, 3);

	for (int i = 0; i < num - 1; i++)
	{
		B.row(i) = M.row(i + 1) - M.row(i);
	}

	Eigen::MatrixXd L2 = Eigen::MatrixXd::Zero(num - 1, 1);
	for (int i = 0; i < num - 1; i++)
	{
		L2(i) = (M(i + 1, 0) * M(i + 1, 0) + M(i + 1, 1) * M(i + 1, 1) + M(i + 1, 2) * M(i + 1, 2)
			- (M(i, 0) * M(i, 0) + M(i, 1) * M(i, 1) + M(i, 2) * M(i, 2))) / 2.0;
	}

	Eigen::Matrix4d D;
	D.setZero();
	D.block<3, 3>(0, 0) = B.transpose() * B;
	D.block<3, 1>(0, 3) = A;
	D.block<1, 3>(3, 0) = A.transpose();

	Eigen::Vector4d L3((B.transpose() * L2)(0), (B.transpose() * L2)(1), (B.transpose() * L2)(2), 1);
	Eigen::Vector4d C = D.inverse() * L3;

	double radius = 0.0;
	for (int i = 0; i < num; i++)
	{
		Eigen::Vector3d tmp(M.row(i)(0) - C(0), M.row(i)(1) - C(1), M.row(i)(2) - C(2));
		radius = radius + sqrt(tmp(0) * tmp(0) + tmp(1) * tmp(1) + tmp(2) * tmp(2));
	}

	radius = radius / num;

	// 保存参数
	coeffs.resize(7);

	// 圆心坐标
	coeffs[0] = C(0);
	coeffs[1] = C(1);
	coeffs[2] = C(2);

	// 圆半径
	coeffs[3] = radius;

	// 平面法向量
	coeffs[4] = A1(0);
	coeffs[5] = A1(1);
	coeffs[6] = A1(2);

}

void fit_3d_circle_ransac(const pcl::PointCloud<PointT>::Ptr cloud, Eigen::VectorXf& coeffs)
{
	//----------------------RANSAC框架----------------------------   
	pcl::SampleConsensusModelCircle3D<PointT>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<PointT>(cloud));
	pcl::RandomSampleConsensus<PointT> ransac(model_circle3D);
	ransac.setDistanceThreshold(0.8);	        // 距离阈值，与模型距离小于0.8的点作为内点
	ransac.setMaxIterations(300);		        // 最大迭代次数
	ransac.computeModel();				        // 拟合3D圆
	ransac.getModelCoefficients(coeffs);
}

//绘制三维空间圆弧
void plot_3d_circle(
	const pcl::PointCloud<PointT>::Ptr in_cloud,
	const Eigen::VectorXf coeffs,
	pcl::PointCloud<PointT>::Ptr& out_cloud) {
	float cx = coeffs[0], cy = coeffs[1], cz = coeffs[2];
	float r = coeffs[3];
	float nx = coeffs[4], ny = coeffs[5], nz = coeffs[6];

	Eigen::Vector3f i(1.0, 0.0, 0.0);
	Eigen::Vector3f j(0.0, 1.0, 0.0);
	Eigen::Vector3f k(0.0, 0.0, 1.0);

	// 平面法向量
	Eigen::Vector3f n(nx, ny, nz);
	Eigen::Vector3f a(0.0, 0.0, 0.0);
	Eigen::Vector3f b(0.0, 0.0, 0.0);

	// 求向量a
	a = n.cross(i);
	if (a.norm() == 0.0)
	{
		a = n.cross(j);
	}

	if (a.norm() == 0.0)
	{
		a = n.cross(k);
	}

	// 求向量 b
	b = n.cross(a);

	// 归一化a，b（圆面两个互垂直向量）
	a.normalize();
	b.normalize();

	//利用空间圆的参数方程生成圆
	PointT Pmin, Pmax;
	pcl::getMinMax3D(*in_cloud, Pmin, Pmax);

	float step = 0.01;
	int num = 360 / step;

	for (int pId = 0; pId < num; pId++)
	{
		float angle = (pId * step / 180.0) * PI;

		PointT Pi;
		Pi.x = cx + r * (a[0] * cos(angle) + b[0] * sin(angle));
		Pi.y = cy + r * (a[1] * cos(angle) + b[1] * sin(angle));
		Pi.z = cz + r * (a[2] * cos(angle) + b[2] * sin(angle));

		//剔除aabb框以外的部分
		if ((Pmin.x <= Pi.x) && (Pi.x <= Pmax.x) &&
			(Pmin.y <= Pi.y) && (Pi.y <= Pmax.y) &&
			(Pmin.z <= Pi.z) && (Pi.z <= Pmax.z))
		{
			out_cloud->push_back(Pi);
		}
	}
}

void
fit_space(
	const pcl::PointCloud<PointT>::Ptr source_cloud,
	const int class_type,
	pcl::PointCloud<PointT>::Ptr& fit_space) {
	switch (class_type)
	{
	case 1: // 空间直线
	{
		Eigen::VectorXf coffes;
		fit_3d_line_ransac(source_cloud, coffes);
		plot_3d_line(source_cloud, coffes, fit_space);
		break;
	}
	case 2: // 空间圆弧
	{
		Eigen::VectorXf coffes;
		fit_3d_circle_ransac(source_cloud, coffes);
		plot_3d_circle(source_cloud, coffes, fit_space);
		break;
	}
	case 3: // 空间曲线
	{
		std::vector<Eigen::VectorXd> coffes;
		fit_3d_curve_ols(source_cloud, coffes);
		plot_3d_curve(source_cloud, coffes, fit_space);

		break;

	}
	default:
		std::cout << "class_type is wrong" << std::endl;
		break;
	}

}

//-*-*-*-*-*-*-*-*-*-*-*-*-后处理-*-*-*-*-*-*-*-*-*-*-*-*-
void
get_line_two_end(
	const pcl::PointCloud<PointT>::Ptr line,
	pcl::PointCloud<PointT>::Ptr &line_out) {

	PointT startPoint;
	PointT firstPoint = line->points[0];//选点云的第一个点

	//遍历直线点集合的点，找到距离第一个点最远的点，作为起点
	float disFS = 0.;
	for (int i = 0; i < line->points.size(); ++i)
	{
		PointT currentPoint = line->points[i];
		//float disCF = pcl::euclideanDistance(currentPoint, startPoint);
		auto disCF = sqrt(pow((currentPoint.x - firstPoint.x), 2) + pow((currentPoint.y - firstPoint.y), 2) + pow((currentPoint.z - firstPoint.z), 2));
		if (disCF >= disFS)
		{
			disFS = disCF;
			startPoint = currentPoint;
		}
	}

	line_out->push_back(startPoint);

	PointT endPoint;
	//遍历直线点集合的点，找到距离起点最远的点，作为终点
	float disSE = 0.;
	for (int i = 0; i < line->points.size(); ++i)
	{
		PointT currentPoint = line->points[i];
		//float disCS = pcl::euclideanDistance(currentPoint, startPoint);
		auto disCS = sqrt(pow((currentPoint.x - startPoint.x), 2) + pow((currentPoint.y - startPoint.y), 2) + pow((currentPoint.z - startPoint.z), 2));
		if (disCS >= disSE)
		{
			disSE = disCS;
			endPoint = currentPoint;
		}
	}

	line_out->push_back(endPoint);
}

// 圆弧的起点、中点 、终点
void
get_cricle_three_end(
	const pcl::PointCloud<PointT>::Ptr circle,
	const Eigen::VectorXf coffes,
	pcl::PointCloud<PointT>::Ptr &cricle_out) {

	double radius = coffes[3];

	int N = circle->points.size();

	// 起点坐标
	PointT startPoint;

	//遍历圆弧点集合的点，找到距离第一个点弧长最远的点，作为起点
	PointT firstPoint = circle->points[0];//选点云的第一个点
	float maxCF = 0.;
	for (int i = 0; i < N; ++i)
	{
		PointT currentPoint = circle->points[i];
		float disCF = sqrt(pow((currentPoint.x - firstPoint.x), 2) + pow((currentPoint.y - firstPoint.y), 2) + pow((currentPoint.z - firstPoint.z), 2));
		float theta = 2 * asin(disCF / (2 * radius));
		float lCF = theta * radius;

		if (lCF >= maxCF)
		{
			maxCF = lCF;
			startPoint = currentPoint;
		}
	}
	cricle_out->push_back(startPoint);

	// 中点坐标
	PointT middlePoint = circle->points[(int)(N / 2)];
	cricle_out->push_back(middlePoint);

	// 终点坐标
	PointT endPoint;
	//遍历直线点集合的点，找到距离起点最远的点，作为终点
	float maxSE = 0.;
	for (int i = 0; i < N; ++i)
	{
		PointT currentPoint = circle->points[i];
		float disSE = sqrt(pow((currentPoint.x - startPoint.x), 2) + pow((currentPoint.y - startPoint.y), 2) + pow((currentPoint.z - startPoint.z), 2));
		float theta = 2 * asin(disSE / (2 * radius));
		float lSE = theta * radius;
		if (lSE >= maxSE)
		{
			maxSE = lSE;
			endPoint = currentPoint;
		}
	}
	cricle_out->push_back(endPoint);

}

void get_curve_step_point(
	const pcl::PointCloud<PointT>::Ptr curve,
	pcl::PointCloud<PointT>::Ptr &curve_out)
{
	for (size_t i = 0; i < curve->points.size(); i++)
	{
		PointT currentPoint = curve->points[i];
		curve->push_back(currentPoint);
	}
}

// 根据检测的类别输出对应的端点
void get_weld_end_point(
	const std::vector<ExtractResult> ext_result,
	ExtractResultL & ext_result_list) {

	for (int i = 0; i < ext_result.size(); ++i)
	{
		int weld_type = 0;
		pcl::PointCloud<PointT>::Ptr weld_cloud(new pcl::PointCloud<PointT>);

		weld_type = ext_result[i].type;
		weld_cloud = ext_result[i].cloud.makeShared();

		pcl::PointCloud<PointT>::Ptr weld_point(new pcl::PointCloud<PointT>);

		ExtractResult extract_result;

		switch (weld_type)
		{
		case 1:
		{
			get_line_two_end(weld_cloud, weld_point);
			break;
		}
		case 2:
		{
			Eigen::VectorXf coffes;
			fit_3d_circle_ransac(weld_cloud, coffes);
			get_cricle_three_end(weld_cloud, coffes, weld_point);
			break;
		}
		case 3:
		{

			get_curve_step_point(weld_cloud, weld_point);
		}

		default:
			break;
		}

		extract_result.type = weld_type;
		extract_result.cloud = *weld_point;
		ext_result_list.push_back(extract_result);
	}

}
