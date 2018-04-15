#include "SubsidiaryFunction.h"

#include <pcl/filters/voxel_grid.h>

void ConvertRGBCloud2XYZCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz)
{
	for (int i = 0; i < rgb->points.size(); ++i)
	{
		float x = rgb->points[i].x;
		float y = rgb->points[i].y;
		float z = rgb->points[i].z;
		pcl::PointXYZ p;
		p.x = x;	p.y = y;	p.z = z;
		xyz->points.push_back(p);
	}
}

void ConvertXYZCloud2RGBCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr xyz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb)
{
	for (int i = 0; i < xyz->points.size(); ++i)
	{
		float x = xyz->points[i].x;
		float y = xyz->points[i].y;
		float z = xyz->points[i].z;
		pcl::PointXYZRGB p;
		p.x = x;	p.y = y;	p.z = z;
		p.r = 255;	p.g = 255;	p.b = 255;
		rgb->points.push_back(p);
	}
}

bool UVD2XYZ(int u, int v, unsigned short d, CameraParam& _camera, double& x, double& y, double& z)
{
	if (d > 0) {
		cv::Mat cam(3, 3, CV_32F);
		cam.at<float>(0, 0) = _camera.fx_; cam.at<float>(0, 1) = 0; cam.at<float>(0, 2) = _camera.cx_;
		cam.at<float>(1, 0) = 0; cam.at<float>(1, 1) = _camera.fy_; cam.at<float>(1, 2) = _camera.cy_;
		cam.at<float>(2, 0) = 0; cam.at<float>(2, 1) = 0; cam.at<float>(2, 2) = 1;
		cv::Mat dist_coef(1, 5, CV_32F);
		dist_coef.at<float>(0, 0) = _camera.k1_;
		dist_coef.at<float>(0, 1) = _camera.k2_;
		dist_coef.at<float>(0, 2) = _camera.p1_;
		dist_coef.at<float>(0, 3) = _camera.p2_;
		dist_coef.at<float>(0, 4) = _camera.k3_;

		cv::Mat mat(1, 2, CV_32F);
		mat.at<float>(0, 0) = u;
		mat.at<float>(0, 1) = v;
		mat = mat.reshape(2);
		cv::undistortPoints(mat, mat, cam, dist_coef, cv::Mat(), cam);
		mat = mat.reshape(1);

		float uu = mat.at<float>(0, 0);
		float vv = mat.at<float>(0, 1);

		z = d / _camera.depth_ratio_;
		x = (uu - _camera.cx_) * z / _camera.fx_;
		y = (vv - _camera.cy_) * z / _camera.fy_;


		// ideal model
		/*z = d / _camera.depth_ratio_;
		x = (u - _camera.cx_) * z / _camera.fx_;
		y = (v - _camera.cy_) * z / _camera.fy_;*/
		return true;
	}
	else {
		return false;
	}
}


pcl::PointXYZRGB  SearchNearestValidPoint(int u, int v, cv::Mat& depth, CameraParam& camera){
	int uu = u, vv = v;
	ushort z1 = depth.at<ushort>(vv, uu);
	double x, y, z;
	int step = 1;
	int forward8 = 0;
	while (!UVD2XYZ(uu, vv, z1, camera, x, y, z) && step<100)
	{
		uu = u;
		vv = v;
		forward8++;
		if (forward8 == 1)
		{
			uu += step;
			if (uu >= depth.cols)
				uu -= step;
			z1 = depth.at<ushort>(vv, uu);
			continue;
		}
		if (forward8 == 2)
		{
			vv -= step;
			if (vv < 0)
				vv += step;
			z1 = depth.at<ushort>(vv, uu);
			continue;
		}
		if (forward8 == 3)
		{
			uu -= step;
			if (uu < 0)
				uu += step;
			z1 = depth.at<ushort>(vv, uu);
			continue;
		}
		if (forward8 == 4)
		{
			vv += step;
			if (vv >= depth.rows)
				vv -= step;
			z1 = depth.at<ushort>(vv, uu);
			continue;
		}
		if (forward8 == 5)
		{
			uu += step;
			vv -= step;
			if (vv < 0 || uu >= depth.cols)
			{
				uu -= step;
				vv += step;
			}
			z1 = depth.at<ushort>(vv, uu);
			continue;
		}
		if (forward8 == 6)
		{
			uu -= step;
			vv -= step;
			if (vv <0  || uu <0)
			{
				uu += step;
				vv += step;
			}
			z1 = depth.at<ushort>(vv, uu);
			continue;
		}
		if (forward8 == 7)
		{
			uu -= step;
			vv += step;
			if (vv >= depth.rows || uu < 0)
			{
				uu += step;
				vv -= step;
			}
			z1 = depth.at<ushort>(vv, uu);
			continue;
		}
		if (forward8 == 8)
		{
			uu += step;
			vv += step;
			if (vv >= depth.rows || uu >= depth.cols)
			{
				uu -= step;
				vv -= step;
			}
			z1 = depth.at<ushort>(vv, uu);
			continue;
		}
		step++;
		forward8 = 0;
	}
	pcl::PointXYZRGB p;
	if (z1==0)
	{
		p.x = x;	p.y = y;	p.z = -1.0f;
		p.r = 255;	p.g = 255;	p.b = 255;
	}
	else
	{
		p.x = x;	p.y = y;	p.z = z;
		p.r = 255;	p.g = 255;	p.b = 255;
	}
	
	return p;
}