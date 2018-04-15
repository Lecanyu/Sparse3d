#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#include <boost/filesystem.hpp>
#include "helper.h"


int main(int argc, char** argv)
{
	std::string pointcloud_ds_dir, narf_traj_file, narf_info_file, sift_traj_file, sift_info_file, corner_traj_file, corner_info_file, traj_file, info_file;
	if (argc != 10)
	{
		std::cout << "Usage: MergeInfo.exe pointcloud_ds_dir narf_traj_file narf_info_file sift_traj_file sift_info_file corner_traj_file corner_info_file traj_file info_file\n";
		return -1;
	}
	//input file
	pointcloud_ds_dir = argv[1];
	narf_traj_file = argv[2];
	narf_info_file = argv[3];
	sift_traj_file = argv[4];
	sift_info_file = argv[5];
	corner_traj_file = argv[6];
	corner_info_file = argv[7];

	// output file
	traj_file = argv[8];
	info_file = argv[9];

	int num_of_pc = std::count_if(
		boost::filesystem::directory_iterator(boost::filesystem::path(pointcloud_ds_dir)),
		boost::filesystem::directory_iterator(),
		[](const boost::filesystem::directory_entry& e) {
		return e.path().extension() == ".pcd";  });

	RGBDInformation narf_info;
	RGBDTrajectory narf_traj;
	RGBDInformation sift_info;
	RGBDTrajectory  sift_traj;
	RGBDInformation corner_info;
	RGBDTrajectory  corner_traj;

	narf_info.LoadFromFile(narf_info_file);
	narf_traj.LoadFromFile(narf_traj_file);
	sift_info.LoadFromFile(sift_info_file);
	sift_traj.LoadFromFile(sift_traj_file);
	corner_info.LoadFromFile(corner_info_file);
	corner_traj.LoadFromFile(corner_traj_file);

	for (int i = 0; i < narf_info.data_.size(); ++i)
		narf_info.data_[i].flag = 0;
	for (int i = 0; i < sift_info.data_.size(); ++i)
		sift_info.data_[i].flag = 1;
	for (int i = 0; i < corner_info.data_.size(); ++i)
		corner_info.data_[i].flag = 2;

	// result
	RGBDInformation info;
	RGBDTrajectory traj;

	for (int i = 0; i < narf_info.data_.size(); ++i)
		info.data_.push_back(narf_info.data_[i]);
	for (int i = 0; i < sift_info.data_.size(); ++i)
		info.data_.push_back(sift_info.data_[i]);
	for (int i = 0; i < corner_info.data_.size(); ++i)
		info.data_.push_back(corner_info.data_[i]);

	for (int i = 0; i < narf_traj.data_.size(); ++i)
		traj.data_.push_back(narf_traj.data_[i]);
	for (int i = 0; i < sift_traj.data_.size(); ++i)
		traj.data_.push_back(sift_traj.data_[i]);
	for (int i = 0; i < corner_traj.data_.size(); ++i)
		traj.data_.push_back(corner_traj.data_[i]);

	std::sort(info.data_.begin(), info.data_.end(), SortInfo());
	std::sort(traj.data_.begin(), traj.data_.end(), SortTraj());

	traj.SaveToFile(traj_file);
	info.SaveToFile(info_file);

	return 0;
}