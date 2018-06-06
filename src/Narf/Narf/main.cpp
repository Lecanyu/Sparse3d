// Registration from depth image by using NARF feature and FPFHSignature33 descriptor
//

#include <vector>
#include <iostream>
#include <string>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh_omp.h>

#include <boost/filesystem.hpp>
#include "NARF.h"
#include "Verbose.h"

int main(int argc, char** argv)
{
	std::string pointcloud_xyzn_dir, keypoint_dir, descriptor_dir;

	if (argc != 4)
	{
		std::cout << "Usage: Narf.exe pointcloud_xyzn_dir keypoint_dir descriptor_dir\n";
		return -1;
	}

	pointcloud_xyzn_dir = argv[1];
	keypoint_dir = argv[2];
	descriptor_dir = argv[3];

	if (!boost::filesystem::exists(keypoint_dir))
		boost::filesystem::create_directory(keypoint_dir);
	if (!boost::filesystem::exists(descriptor_dir))
		boost::filesystem::create_directory(descriptor_dir);


	int num_of_pcd = std::count_if(
		boost::filesystem::directory_iterator(boost::filesystem::path(pointcloud_xyzn_dir)),
		boost::filesystem::directory_iterator(),
		[](const boost::filesystem::directory_entry& e) {
		return e.path().extension() == ".pcd";  });


	for (int i = 0; i < num_of_pcd;++i)
	{
		// normal point cloud
		std::stringstream nss;
		nss << pointcloud_xyzn_dir << "pointcloud_xyzn" << i << ".pcd";
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene_xyzn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::io::loadPCDFile(nss.str(), *scene_xyzn);
		pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
		for (int t = 0; t < scene_xyzn->points.size(); ++t)
		{
			pcl::Normal n;
			n.normal_x = scene_xyzn->points[t].normal_x;
			n.normal_y = scene_xyzn->points[t].normal_y;
			n.normal_z = scene_xyzn->points[t].normal_z;
			normal->push_back(n);
		}

		// point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		for (int t = 0; t < scene_xyzn->points.size(); ++t)
		{
			pcl::PointXYZ p;
			p.x = scene_xyzn->points[t].x;
			p.y = scene_xyzn->points[t].y;
			p.z = scene_xyzn->points[t].z;
			scene_xyz->push_back(p);
		}

		// keypoint
#ifdef Verbose
		std::cout << "Detect pointcloud" << i << " keypoint\n";
#endif
		NARF narf(scene_xyz);
		narf.narf_keypoints_extraction();

		if (narf.keypoints.size() <= 0)
			continue;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(narf.keypoints, *pointcloud_keypoints);
		pointcloud_keypoints->width = pointcloud_keypoints->size();
		pointcloud_keypoints->height = 1;

		// descriptor
#ifdef Verbose
		std::cout << "Detect pointcloud" << i << " descriptor\n";
#endif
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor(new pcl::PointCloud<pcl::FPFHSignature33>());

		pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud(pointcloud_keypoints);
		fpfh.setSearchSurface(scene_xyz);
		fpfh.setInputNormals(normal);
		fpfh.setRadiusSearch(0.06);
		fpfh.compute(*descriptor);

		//save
		if (pointcloud_keypoints->points.size() > 100){
			pointcloud_keypoints->points.resize(100);
			descriptor->points.resize(100);
			pointcloud_keypoints->width = pointcloud_keypoints->points.size();
		}		
		std::stringstream kss;
		kss << keypoint_dir << "keypoints" << i << ".pcd";
		pcl::io::savePCDFile(kss.str(), *pointcloud_keypoints);

		std::stringstream dss;
		dss << descriptor_dir << "descriptor" << i << ".txt";
		std::ofstream d_file(dss.str());
		d_file << "Descriptor vector (FPFHSignature33):\n";
		for (int k = 0; k < descriptor->points.size(); ++k)
		{
			d_file << "keypoint" << k << "\n";
			for (int t = 0; t < 33; ++t)
				d_file << descriptor->points[k].histogram[t] << " ";

			d_file << "\n";
		}
			
	}


	return 0;
}