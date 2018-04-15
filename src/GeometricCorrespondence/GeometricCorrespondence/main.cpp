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
	std::string pointcloud_dir, pointcloud_ds_dir, keypoint_dir, descriptor_dir, camera_file, traj_file, info_file;

	if (argc != 9)
	{
	std::cout << "Usage:\n\GeometricCorrespondence.exe pointcloud_dir[input_dir] pointcloud_ds_dir[input_dir] keypoint_dir[input_dir] descriptor_dir[input_dir] camera_file[input_file] score_max_depth[<0: will ignore]"
	<< " traj_file[output_file] info_file[output_file]\n";
	return 0;
	}

	pointcloud_dir = argv[1];
	pointcloud_ds_dir = argv[2];
	keypoint_dir = argv[3];
	descriptor_dir = argv[4];
	camera_file = argv[5];
	score_max_depth = atof(argv[6]);

	traj_file = argv[7];
	info_file = argv[8];


	int num_of_pc = std::count_if(
		boost::filesystem::directory_iterator(boost::filesystem::path(pointcloud_dir)),
		boost::filesystem::directory_iterator(),
		[](const boost::filesystem::directory_entry& e) {
		return e.path().extension() == ".pcd";  });

	// Load all downsample pointcloud
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> downsample_pc;
	for (int i = 0; i < num_of_pc; ++i)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_ds(new pcl::PointCloud<pcl::PointXYZRGB>);

		std::stringstream dsss1;
		dsss1 << pointcloud_ds_dir << "pointcloud_ds" << i << ".pcd";
		pcl::io::loadPCDFile(dsss1.str(), *scene_ds);
		downsample_pc.push_back(scene_ds);
	}

	// Load all keypoints
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> keypoints(num_of_pc);
	for (int i = 0; i < num_of_pc;++i)
	{
		std::stringstream pkss1;
		pkss1 << keypoint_dir << "keypoints" << i << ".pcd";
		if (!boost::filesystem::exists(pkss1.str()))
			continue;
		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile(pkss1.str(), *keypoint);
		keypoints[i] = keypoint;
	}
	
	// Load all descriptors
	std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> descriptors(num_of_pc);
	for (int i = 0; i < num_of_pc;++i)
	{
		ImageDescriptor dp;
		std::stringstream dpss;
		dpss << descriptor_dir << "descriptor" << i << ".txt";
		if (!boost::filesystem::exists(dpss.str()))
			continue;

		dp.LoadFromFile(dpss.str());
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor(new pcl::PointCloud<pcl::FPFHSignature33>());
		for (int t = 0; t < dp.data_.size(); ++t)
		{
			pcl::FPFHSignature33 d;
			for (int k = 0; k < dp.data_[t].dvec.size(); ++k)
				d.histogram[k] = dp.data_[t].dvec[k];
			descriptor->points.push_back(d);
		}
		descriptors[i] = descriptor;
	}

	RGBDInformation info;	info.data_.resize(num_of_pc*num_of_pc);
	RGBDTrajectory  traj;	traj.data_.resize(num_of_pc*num_of_pc);

	// deal with corner correspondence
	if (boost::filesystem::exists(traj_file))
		boost::filesystem::remove(traj_file);
	if (boost::filesystem::exists(info_file))
		boost::filesystem::remove(info_file);
#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
	for (int i = 0; i < num_of_pc; ++i)
	{
		for (int j = i + 1; j < num_of_pc; ++j)
		{
			std::stringstream ssf1, ssf2;
			ssf1 << keypoint_dir << "keypoints" << i << ".pcd";
			ssf2 << keypoint_dir << "keypoints" << j << ".pcd";

			if (!boost::filesystem::exists(ssf1.str()) || !boost::filesystem::exists(ssf2.str()))
				continue;

			int img1 = i;
			int img2 = j;

			std::cout << "Begin registration geometric correspondence between (" << img1 << ", " << img2 << ")\n";

			pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint1 = keypoints[img1];
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor1 = descriptors[img1];

			pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint2 = keypoints[img2];
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor2 = descriptors[img2];

			if (keypoint1->size() < 4 || keypoint2->size() < 4)
				continue;

			// guess correspondence
			std::cout << "Guess correspondences...";
			pcl::CorrespondencesPtr corps(new pcl::Correspondences());
			pcl::KdTreeFLANN<pcl::FPFHSignature33> match_search;
			match_search.setInputCloud(descriptor2);
			for (int t = 0; t < descriptor1->size(); ++t)
			{
				int N_nearest = 2;

				std::vector<int> neigh_indices(N_nearest);
				std::vector<float> neigh_sqr_dists(N_nearest);

				match_search.nearestKSearch(descriptor1->at(t), N_nearest, neigh_indices, neigh_sqr_dists);

				if (neigh_sqr_dists[0] < 0.8*neigh_sqr_dists[1])
				{
					pcl::Correspondence corr(t, neigh_indices[0], neigh_sqr_dists[0]);
					corps->push_back(corr);
				}
			}
			std::cout << "Done!\n";

			if (corps->size() < 4)
				continue;

			//Graph matching
			std::cout << "Graph matching...";
			GraphMatching gm(*keypoint1, *keypoint2, *corps);
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