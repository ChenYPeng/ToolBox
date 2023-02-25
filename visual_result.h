#pragma once
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "utils.h"


//可视化点云
void vis1cloud(const pcl::PointCloud<PointT>::Ptr cloud, const std::string &viewerName);
void vis2cloud(const pcl::PointCloud<PointT>::Ptr cloud1, const pcl::PointCloud<PointT>::Ptr cloud2);
void vis3cloud(const pcl::PointCloud<PointT>::Ptr cloud1, const pcl::PointCloud<PointT>::Ptr cloud2, const pcl::PointCloud<PointT>::Ptr cloud3);

template <typename MatrixT>
void printMatrix4x4(const MatrixT matrix, std::string file_name)
{
	printf("%s :\n", file_name.c_str());
	printf("    | %9.6f %9.6f %9.6f %12.6f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3));
	printf("T = | %9.6f %9.6f %9.6f %12.6f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3));
	printf("    | %9.6f %9.6f %9.6f %12.6f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3));
	printf("    | %9.6f %9.6f %9.6f %12.6f | \n", matrix(3, 0), matrix(3, 1), matrix(3, 2), matrix(3, 3));
	printf("\n");
}

template <typename T>
void printPointCloud(const typename pcl::PointCloud<T> cloud, const std::string &file_name)
{
	printf("%s :\n", file_name.c_str());
	for (size_t i = 0; i < cloud.points.size(); i++)
	{
		std::cout << cloud.points[i] << std::endl;
	}
	printf("\n");
}

void vis1cloud(const pcl::PointCloud<PointT>::Ptr cloud, const std::string &viewerName)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<PointT>(cloud, "cloud");
	viewer->setWindowName(viewerName.c_str());

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(10);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}
	viewer->close();

}

void vis2cloud(const pcl::PointCloud<PointT>::Ptr cloud1, const pcl::PointCloud<PointT>::Ptr cloud2)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<PointT>(cloud1, "cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud1");

	viewer->addPointCloud<PointT>(cloud2, "cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud2");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(10);
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
	viewer->close();
}


void vis3cloud(const pcl::PointCloud<PointT>::Ptr cloud1, const pcl::PointCloud<PointT>::Ptr cloud2, const pcl::PointCloud<PointT>::Ptr cloud3)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<PointT>(cloud1, "cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud1");

	viewer->addPointCloud<PointT>(cloud2, "cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud2");

	viewer->addPointCloud<PointT>(cloud3, "cloud3");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud3");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud3");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(10);
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
	viewer->close();
}