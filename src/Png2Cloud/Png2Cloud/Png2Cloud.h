#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>

#include "Verbose.h"

struct CameraParam {
public:
	double fx_, fy_, cx_, cy_;
	double k1_, k2_, k3_, p1_, p2_;
	double depth_ratio_;
	double downsample_leaf;

	int img_width_;
	int img_height_;

	double integration_trunc_;
	CameraParam() :
		fx_(525.0f), fy_(525.0f), cx_(319.5f), cy_(239.5f),
		k1_(0.0), k2_(0.0), k3_(0.0), p1_(0.0), p2_(0.0),
		depth_ratio_(1000.0),
		downsample_leaf(0.05),
		img_width_(640), img_height_(480),
		integration_trunc_(4.0)
	{}

	void LoadFromFile(std::string filename) {
		FILE * f = fopen(filename.c_str(), "r");
		if (f != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%lf", &fx_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &fy_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &cx_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &cy_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &k1_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &k2_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &k3_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &p1_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &p2_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &depth_ratio_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &downsample_leaf);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%d", &img_width_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%d", &img_height_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &integration_trunc_);
				}
			}
			printf("Camera model set to (fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f, k1: %.2f, k2: %.2f, k3: %.2f, p1: %.2f, p2: %.2f, depth_ratio_: %.2f, downsample_leaf: %.2f, img_width_: %d, img_height_: %d, integration_trunc: %.2f)\n",
				fx_, fy_, cx_, cy_, k1_, k2_, k3_, p1_, p2_, depth_ratio_, downsample_leaf, img_width_, img_height_, integration_trunc_);
			fclose(f);
		}
	}
};


class Png2Cloud
{
public:
	CameraParam _camera;

	bool Load(int png_index, std::string& depth_dir, std::string& color_dir, cv::Mat& depth_img, cv::Mat& color_img)
	{
		std::stringstream ss;
		ss << depth_dir  <<"depth"<< png_index << ".png";
		std::string filepath_depth = ss.str();

		std::stringstream sss;
		sss << color_dir  <<"color"<< png_index << ".png";
		std::string filepath_color = sss.str();


		depth_img = cv::imread(filepath_depth, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
		depth_img.convertTo(depth_img, CV_16U);
#ifdef Verbose
		std::cout << "depth image resolution: " << depth_img.cols << "*" << depth_img.rows << std::endl;
#endif // Verbose
		assert(_camera.img_width_ == depth_img.cols && _camera.img_height_ == depth_img.rows);

		color_img = cv::imread(filepath_color, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
		color_img.convertTo(color_img, CV_8UC3);
#ifdef Verbose
		std::cout << "color image resolution: " << color_img.cols << "*" << color_img.rows << std::endl;
#endif
		return true;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Png2RGBcloud(cv::Mat& depth_img, cv::Mat& color_img)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int i = 0; i < depth_img.cols; ++i)		//i => u
		{
			for (int j = 0; j < depth_img.rows; ++j)	//j => v
			{
				double x, y, z;
				if (UVD2XYZ(i, j, depth_img.at<ushort>(j, i), x, y, z))
				{
					// color pointcloud
					pcl::PointXYZRGB rgb;
					rgb.x = x;	rgb.y = y;	rgb.z = z;

					cv::Vec3b intensity = color_img.at<cv::Vec3b>(j, i);
					rgb.b = intensity.val[0];
					rgb.g = intensity.val[1];
					rgb.r = intensity.val[2];
					cloud->points.push_back(rgb);
				}

			}
		}

		return cloud;
	}

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

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ComputeModelNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, float radius=0.05)
	{
		radius = _camera.downsample_leaf;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		// convert rgb cloud to xyz cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		ConvertRGBCloud2XYZCloud(scene, scene_xyz);


		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
		norm_est.setRadiusSearch(radius);
		norm_est.setInputCloud(scene_xyz);
		norm_est.compute(*normals);

		assert(normals->points.size() == scene->points.size());
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		for (int i = 0; i < scene->points.size(); ++i)
		{
			pcl::PointXYZRGBNormal p;
			p.x = scene->points[i].x;
			p.y = scene->points[i].y;
			p.z = scene->points[i].z;
			p.r = scene->points[i].r;
			p.g = scene->points[i].g;
			p.b = scene->points[i].b;
			p.normal_x = normals->points[i].normal_x;
			p.normal_y = normals->points[i].normal_y;
			p.normal_z = normals->points[i].normal_z;
			result->points.push_back(p);
		}
		return result;
	}


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr DownSamle(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::VoxelGrid<pcl::PointXYZRGB> grid;
		float leaf = _camera.downsample_leaf;
		grid.setLeafSize(leaf, leaf, leaf);
		grid.setInputCloud(cloud_in);
		grid.filter(*cloud_out);
		return cloud_out;
	}


	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr DownSamle(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud_in)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
		float leaf = _camera.downsample_leaf;
		grid.setLeafSize(leaf, leaf, leaf);
		grid.setInputCloud(cloud_in);
		grid.filter(*cloud_out);
		return cloud_out;
	}


	int round(double x) {
		return static_cast<int>(floor(x + 0.5));
	}

	bool UVD2XYZ(int u, int v, unsigned short d, double & x, double & y, double & z) {
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

	bool XYZ2UVD(double x, double y, double z, int & u, int & v, unsigned short & d) {
		if (z > 0) {
			float x_ = x / z;	float y_ = y / z;
			float r2 = x_*x_ + y_*y_;
			float x__ = x_*(1 + _camera.k1_*r2 + _camera.k2_*r2*r2 + _camera.k3_*r2*r2*r2) + 2 * _camera.p1_*x_*y_ + _camera.p2_*(r2 + 2 * x_*x_);
			float y__ = y_*(1 + _camera.k1_*r2 + _camera.k2_*r2*r2 + _camera.k3_*r2*r2*r2) + _camera.p1_*(r2 + 2 * y_*y_) + 2 * _camera.p2_*x_*y_;
			float uu = x__ * _camera.fx_ + _camera.cx_;
			float vv = y__ * _camera.fy_ + _camera.cy_;
			u = round(uu);
			v = round(vv);
			d = static_cast<unsigned short>(round(z * _camera.depth_ratio_));

			// ideal model
			/*u = round(x * _camera.fx_ / z + _camera.cx_);
			v = round(y * _camera.fy_ / z + _camera.cy_);
			d = static_cast<unsigned short>(round(z * _camera.depth_ratio_));*/
			return (u >= 0 && u < _camera.img_width_ && v >= 0 && v < _camera.img_height_);
		}
		else {
			return false;
		}
	}
};