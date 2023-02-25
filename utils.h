#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <chrono> //time

#define PI acos(-1)

typedef pcl::PointXY Point2D;
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNormalT;

typedef std::vector<pcl::PointCloud<PointT>> PointCloudL;
typedef std::vector<PointCloudL> PointCloudLL;

typedef struct ExtractResult
{
	int type;
	pcl::PointCloud<PointT> cloud;
}ExtractResult;

typedef std::vector<ExtractResult> ExtractResultL;		// ��ŵ�֡���
typedef std::vector<ExtractResultL> ExtractResultLL;	// ��Ŷ�֡���

typedef struct CameraParam
{
	float cx;
	float cy;
	float fx;
	float fy;
	int width;
	int height;
	int focalLength;

}CameraParam;

typedef struct EulerAngle
{
	double rx, ry, rz; //����
}EulerAngle;

typedef struct Quaternion
{
	double w, x, y, z;
}Quaternion;

typedef struct ToolPose
{
	double x, y, z, rx, ry, rz;
}ToolPose;
