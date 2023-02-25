#pragma once
#include <vtkSTLReader.h>
#include <vtkPolyData.h>
#include <vtkConeSource.h>				// 源数据
#include <vtkPolyDataMapper.h>			// 数据映射
#include <vtkRenderWindow.h>			// 绘制窗口
#include <vtkRenderWindowInteractor.h>	// 交互机制
#include <vtkRenderer.h>				// 绘制器
#include <vtkActor.h>					// 演员
#include <vtkCamera.h>					// 相机
#include <vtkMatrix4x4.h>
#include <vtkWindowToImageFilter.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkWorldPointPicker.h>

#include "utils.h"

class MeshToPoint
{
private:
	CameraParam cameraParam;

public:
	MeshToPoint();

	void setCameral(CameraParam camera);

	void getCamera(double *cx, double *cy, double *fx, double *fy, int *width, int *height, int *focalLength);

	void getSurfacePoint(
		const vtkSmartPointer<vtkPolyData> polyData,
		const Eigen::Matrix4f icpMatrix,
		const Eigen::Matrix4f handeye,
		const Eigen::Matrix4f toolpose,
		Eigen::Matrix4f &viewPose,
		pcl::PointCloud<PointT>::Ptr &cloud_out);

};


MeshToPoint::MeshToPoint()
{
	cameraParam.cx = 654.476;
	cameraParam.cy = 483.953;
	cameraParam.fx = 1175.98;
	cameraParam.fy = 1176.33;

	cameraParam.width = 1280;
	cameraParam.height = 1024;
	cameraParam.focalLength = 1;
}

void MeshToPoint::setCameral(CameraParam camera)
{
	cameraParam = camera;
}

void MeshToPoint::getCamera(double *cx, double *cy, double *fx, double *fy, int *width, int *height, int *focalLength)
{
	*cx = MeshToPoint::cameraParam.cx;
	*cy = MeshToPoint::cameraParam.cy;
	*fx = MeshToPoint::cameraParam.fx;
	*fy = MeshToPoint::cameraParam.fy;

	*width = cameraParam.width;
	*height = cameraParam.height;
	*focalLength = cameraParam.focalLength;

}

void MeshToPoint::getSurfacePoint(
	const vtkSmartPointer<vtkPolyData> polyData,
	const Eigen::Matrix4f icpMatrix,
	const Eigen::Matrix4f handEye,
	const Eigen::Matrix4f toolPose,
	Eigen::Matrix4f &viewPose,
	pcl::PointCloud<PointT>::Ptr &cloud_out) {

	// Matrix4x4 转 vtkMatrix4x4
	Eigen::Matrix4d icpMatrix4d = icpMatrix.cast<double>();
	double *icpArray = new double[icpMatrix.size()];
	Eigen::Map<Eigen::MatrixXd>(icpArray, icpMatrix4d.rows(), icpMatrix4d.cols()) = icpMatrix4d.transpose();
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(icpArray);

	// 计算视点
	Eigen::Matrix4f camera2base;
	camera2base = toolPose * handEye;
	viewPose = camera2base.inverse();

	// 实例化映射器
	vtkSmartPointer<vtkPolyDataMapper> cubeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	cubeMapper->SetInputData(polyData);
	cubeMapper->Update();

	// 创建演员对象
	vtkSmartPointer<vtkActor> cubeActor = vtkSmartPointer<vtkActor>::New();
	cubeActor->SetMapper(cubeMapper);
	cubeActor->SetUserMatrix(matrix);

	vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();

	// 设定相机的内参
	double cx, cy, fx, fy;
	int width, height, focalLength;
	MeshToPoint::getCamera(&cx, &cy, &fx, &fy, &width, &height, &focalLength);

	// 设定相机的外参
	Eigen::Matrix3f R;
	Eigen::Vector3f t;
	R = viewPose.block<3, 3>(0, 0);
	t = viewPose.block<3, 1>(0, 3);

	// 剪切平面
	camera->SetClippingRange(0.1, 1000);

	// 相机位置
	Eigen::Vector3f position = -R.transpose() * t;
	camera->SetPosition(position[0], position[1], position[2]);
	//std::cout << "Set Camera Position: [" << position[0] << "," << position[1] << "," << position[2] << "]" << std::endl;

	// 焦点位置
	Eigen::Vector3f focalPoint = position + R.row(2).transpose() * focalLength;
	camera->SetFocalPoint(focalPoint[0], focalPoint[1], focalPoint[2]);
	//std::cout << "Set Camera Focal Point: [" << focalPoint[0] << "," << focalPoint[1] << "," << focalPoint[2] << "]" << std::endl;

	// 朝上方向
	Eigen::Vector3f viewUp = -R.row(1);
	camera->SetViewUp(viewUp[0], viewUp[1], viewUp[2]);
	//std::cout << "Set Camera View Up: [" << viewUp[0] << "," << viewUp[1] << "," << viewUp[2] << "]" << std::endl;

	// 将主点转化为归一化的图像坐标系
	double wcx = -2 * (cx - double(width) / 2) / width;
	double wcy = 2 * (cy - double(height) / 2) / height;
	camera->SetWindowCenter(wcx, wcy);

	// 将焦距转化为视场角
	double viewAngle = (2.0 * std::atan2(height / 2.0, fy)) * 180. / PI;
	camera->SetViewAngle(viewAngle);
	//std::cout << "Set Camera View Angle: " << viewAngle << std::endl;

	camera->Modified();

	// 搭建舞台
	vtkRenderer *renderer = vtkRenderer::New();
	renderer->AddActor(cubeActor);
	renderer->SetBackground(0, 0, 0);
	renderer->SetActiveCamera(camera);
	renderer->ResetCameraClippingRange();

	// 创建窗口对象
	vtkRenderWindow *renderWindow = vtkRenderWindow::New();
	renderWindow->SetSize(width, height);
	renderWindow->AddRenderer(renderer);
	renderWindow->SetOffScreenRendering(1);
	renderWindow->Render();

	//将当前窗口保存为3D点云
	vtkSmartPointer<vtkWorldPointPicker> worldpicker = vtkSmartPointer<vtkWorldPointPicker>::New();
	double coords[3];
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			double z = renderWindow->GetZbufferDataAtPoint(x, y);
			if (z == 1)
			{
				continue;
			}
			else
			{
				worldpicker->Pick(x, y, z, renderer);
				worldpicker->GetPickPosition(coords);
				PointT tmp;
				tmp.x = coords[0];
				tmp.y = coords[1];
				tmp.z = coords[2];
				cloud_out->push_back(tmp);
			}
		}
	}

}


