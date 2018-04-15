// G2O optimize

#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>

#include "OptApp.h"

int main(int argc, char** argv)
{
	std::string init_pose_file, pointcloud_dir, loop_file, loop_info_file, pose_file, loop_remain_file, info_remain_file;
	double weight;
	if (argc != 9)
	{
		std::cout << "Usage:\n\GlobalOptimizer2.exe init_pose_file[input_file] pointcloud_dir[input_dir] loop_file[input_file] loop_info_file[input_file] pose_file[output_file] loop_remain_file[output_file] info_remain_file[output_file] weight[double number, the more larger, the more edges will be save]\n";
		return -1;
	}

	init_pose_file = argv[1];
	pointcloud_dir = argv[2];
	loop_file = argv[3];
	loop_info_file = argv[4];
	pose_file = argv[5];
	loop_remain_file = argv[6];
	info_remain_file = argv[7];
	weight = atof(argv[8]);

	int num_of_pc = std::count_if(
		boost::filesystem::directory_iterator(boost::filesystem::path(pointcloud_dir)),
		boost::filesystem::directory_iterator(),
		[](const boost::filesystem::directory_entry& e) {
		return e.path().extension() == ".pcd";  });

	COptApp opt(num_of_pc, init_pose_file, loop_file, loop_info_file, pose_file, loop_remain_file, info_remain_file, weight);

	if (opt.Init())
		opt.OptimizeSwitchable();

	return 0;
}