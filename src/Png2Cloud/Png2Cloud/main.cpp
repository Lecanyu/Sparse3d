// Preprocessing. Convert images to point clouds, calculate normals and etc... 
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <boost/filesystem.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/io/pcd_io.h>

#include "Png2Cloud.h"


int main(int argc, char**argv)
{
	if (argc != 8)
	{
		std::cout << "Usage: Png2Cloud.exe depth_dir color_dir camera_para pointcloud_dir pointcloud_ds_dir xyzn_dir xyzn_ds_dir\n";
		return -1;
	}
	std::string depth_dir = argv[1];
	std::string color_dir = argv[2];
	std::string camera_para = argv[3];
	std::string pointcloud_dir = argv[4];
	std::string pointcloud_ds_dir = argv[5];
	std::string xyzn_dir = argv[6];
	std::string xyzn_ds_dir = argv[7];

	if (!boost::filesystem::exists(pointcloud_dir))
		boost::filesystem::create_directory(pointcloud_dir);
	if (!boost::filesystem::exists(pointcloud_ds_dir))
		boost::filesystem::create_directory(pointcloud_ds_dir);
	if (!boost::filesystem::exists(xyzn_dir))
		boost::filesystem::create_directory(xyzn_dir);
	if (!boost::filesystem::exists(xyzn_ds_dir))
		boost::filesystem::create_directory(xyzn_ds_dir);

	int num_of_depth = std::count_if(
		boost::filesystem::directory_iterator(boost::filesystem::path(depth_dir)),
		boost::filesystem::directory_iterator(),
		[](const boost::filesystem::directory_entry& e) {
		return e.path().extension() == ".png";  });

	int num_of_color = std::count_if(
		boost::filesystem::directory_iterator(boost::filesystem::path(color_dir)),
		boost::filesystem::directory_iterator(),
		[](const boost::filesystem::directory_entry& e) {
		return e.path().extension() == ".png";  });

	Png2Cloud app;
	app._camera.LoadFromFile(camera_para);

	for (int i = 0; i < num_of_color;++i)
	{
#ifdef Verbose
		std::cout << "load png " << i << "\n";
#endif // Verbose
		// load
		cv::Mat color;
		cv::Mat depth;
		app.Load(i, depth_dir, color_dir, depth, color);

		// Png to RGBcloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = app.Png2RGBcloud(depth, color);
		std::stringstream ss1;
		ss1 << pointcloud_dir << "pointcloud" << i << ".pcd";
		pcl::io::savePCDFile(ss1.str(), *cloud, true);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ds = app.DownSamle(cloud);
		std::stringstream ss2;
		ss2 << pointcloud_ds_dir << "pointcloud_ds" << i << ".pcd";
		pcl::io::savePCDFile(ss2.str(), *cloud_ds, true);

		// compute normal
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raw_cloud_n = app.ComputeModelNormal(cloud);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		for (int j = 0; j < raw_cloud_n->points.size(); j++) {
			if (!_isnan(raw_cloud_n->points[j].normal_x)) {
				cloud_n->push_back(raw_cloud_n->points[j]);
			}
		}
		cloud_n->width = cloud_n->points.size();
		cloud_n->height = 1;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n_ds = app.DownSamle(cloud_n);

		std::stringstream ss3;
		ss3 << xyzn_dir << "pointcloud_xyzn" << i << ".pcd";
		pcl::io::savePCDFile(ss3.str(), *cloud_n, true);

		std::stringstream ss4;
		ss4 << xyzn_ds_dir << "pointcloud_ds_xyzn" << i << ".pcd";
		pcl::io::savePCDFile(ss4.str(), *cloud_n_ds, true);
		
	}
	return 0;
}