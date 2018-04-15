#include <vector>
#include <iostream>
#include <string>
#include <set>

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/correspondence.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp> 

#include <boost/filesystem.hpp>

#include "SubsidiaryFunction.h"
#include "GraphMatching.h"
#include "BuildCorpPointSet.h"

int main(int argc, char** argv)
{
	double score_max_depth;
	std::string pointcloud_dir, pointcloud_ds_dir, depth_dir, correspendence_file, traj_file, info_file, camera_file;

	if (argc != 9)
	{
		std::cout << "Usage: \nColorCorrespondence.exe pointcloud_dir[input_dir] pointcloud_ds_dir[input_dir] depth_dir[input_dir] correspendence_file[input_file] camera_file[input_file] score_max_depth[<0: will ignore]"
			<< " traj_file[output_file] info_file[output_file]\n";
		return 0;
	}

	pointcloud_dir = argv[1];
	pointcloud_ds_dir = argv[2];
	depth_dir = argv[3];
	correspendence_file = argv[4];
	camera_file = argv[5];
	score_max_depth = atof(argv[6]);

	traj_file = argv[7];
	info_file = argv[8];


	int num_of_pc = std::count_if(
		boost::filesystem::directory_iterator(boost::filesystem::path(depth_dir)),
		boost::filesystem::directory_iterator(),
		[](const boost::filesystem::directory_entry& e) {
		return e.path().extension() == ".png";  });

	// CameraParam
	CameraParam camera;
	camera.LoadFromFile(camera_file);

	CorrespondencePixel cor_pixel;
	cor_pixel.LoadFromFile(correspendence_file);

	// Load all downsample pointcloud
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> downsample_pc;
	for (int i = 0; i < num_of_pc;++i)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_ds(new pcl::PointCloud<pcl::PointXYZRGB>);

		std::stringstream dsss1;
		dsss1 << pointcloud_ds_dir << "pointcloud_ds" << i << ".pcd";
		pcl::io::loadPCDFile(dsss1.str(), *scene_ds);
		downsample_pc.push_back(scene_ds);
	}

	// Load all depth images
	std::vector<cv::Mat> depth_img;
	for (int i = 0; i < num_of_pc;++i)
	{
		std::stringstream ss1;
		ss1 << depth_dir << "depth" << i << ".png";
		cv::Mat depth_image = cv::imread(ss1.str(), CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
		depth_image.convertTo(depth_image, CV_16U);
		depth_img.push_back(depth_image);
	}

	RGBDInformation info;	info.data_.resize(num_of_pc*num_of_pc);
	RGBDTrajectory  traj;	traj.data_.resize(num_of_pc*num_of_pc);

	if (boost::filesystem::exists(traj_file))
		boost::filesystem::remove(traj_file);
	if (boost::filesystem::exists(info_file))
		boost::filesystem::remove(info_file);

#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
	for (int i = 0; i < cor_pixel.data_.size(); ++i)
	{
		int img1 = cor_pixel.data_[i].imageid1_;
		int img2 = cor_pixel.data_[i].imageid2_;

		std::cout << "Begin registration correspondence between (" << img1 << ", " << img2 << ")\n";

		cv::Mat& depth_image1 = depth_img[img1];
		cv::Mat& depth_image2 = depth_img[img2];

		// prepare
		pcl::CorrespondencesPtr corps(new pcl::Correspondences);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_keypoints1(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_keypoints2(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (int t = 0; t < cor_pixel.data_[i].pixel_correspondence_.size(); ++t)
		{
			int image1_u = cor_pixel.data_[i].pixel_correspondence_[t].image1_pixel_x;
			int image1_v = cor_pixel.data_[i].pixel_correspondence_[t].image1_pixel_y;
			pcl::PointXYZRGB p1 = SearchNearestValidPoint(image1_u, image1_v, depth_image1, camera);

			int image2_u = cor_pixel.data_[i].pixel_correspondence_[t].image2_pixel_x;
			int image2_v = cor_pixel.data_[i].pixel_correspondence_[t].image2_pixel_y;
			pcl::PointXYZRGB p2 = SearchNearestValidPoint(image2_u, image2_v, depth_image2, camera);

			if (p1.z < 0.0 || p2.z < 0.0)
				continue;

			pointcloud_keypoints1->push_back(p1);
			pointcloud_keypoints2->push_back(p2);

			pcl::Correspondence c;
			c.index_query = pointcloud_keypoints1->size() - 1;	c.index_match = pointcloud_keypoints2->size() - 1;
			corps->push_back(c);
		}
		if (corps->size() < 4)
			continue;

		//Graph matching
		std::cout << "Graph matching...";
		GraphMatching gm(*pointcloud_keypoints1, *pointcloud_keypoints2, *corps);
		pcl::CorrespondencesPtr	graph_corps = gm.ComputeCorrespondenceByEigenVec();
		std::cout << "Done!\n";

		if (graph_corps->size() < 4)
		{
			std::cout << "cannot find enough correspondence\n";
			continue;
		}

		Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

		// result
		transformation_matrix = transformation_matrix*gm._transformation;

		//////////////////////////
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene1_ds = downsample_pc[img1];
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene2_ds = downsample_pc[img2];

		BuildCorpPointSet buidCorp;
		int K_set = buidCorp.ComputeCorrepondencePointSet(transformation_matrix, scene1_ds, scene2_ds, score_max_depth);

		if (K_set < buidCorp._total_size / 3 || K_set == 0 || buidCorp._total_size == 0)
		{
			std::cout << "Align fail!\n\n\n";
			continue;
		}

		Eigen::Matrix< double, 6, 6 > info_mat = buidCorp.ComputeInfoMatrix(buidCorp._correspondences, scene1_ds, scene2_ds);

		std::cout << K_set << " point pair are found as correspondence in point cloud. " << std::endl;
		std::cout << "Align successfully!\n\n";

		double score = (double)K_set / (double)buidCorp._total_size;

		InformationMatrix im = info_mat;
		FramedInformation fi(img1, img2, im, score);
		info.data_[img1*num_of_pc + img2] = fi;

		Eigen::Matrix4d tf = transformation_matrix.cast<double>();
		FramedTransformation ft(img1, img2, tf);
		traj.data_[img1*num_of_pc + img2] = ft;
	}

	//save
	std::cout << "Save trajectory file...";
	traj.SaveToSPFile(traj_file);
	std::cout << "Done!\n";

	std::cout << "Save information file...";
	info.SaveToSPFile(info_file);
	std::cout << "Done!\n";

	return 0;
}