#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

class BuildCorpPointSet
{
private:
	float _corr_dist_threshold;

public:

	// correspondences source<->target
	pcl::Correspondences _correspondences;
	pcl::Correspondences _final_correspondences;

	int _total_size;

	BuildCorpPointSet() :_corr_dist_threshold(0.075) , _total_size(0)
	{}



	// return correpondence number and correspondences point set
	int ComputeCorrepondencePointSet(const Eigen::Matrix4f& transform, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene2, double score_max_depth)
	{
		_correspondences.clear();
		_correspondences.reserve(scene2->points.size());
		Eigen::Affine3f affine(transform);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*scene1, *scene, affine);


		pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;
		match_search.setInputCloud(scene2);

		int total_size = 0;

		const float max_range = _corr_dist_threshold * _corr_dist_threshold;
		for (int t = 0; t < scene->points.size(); ++t)
		{	
			if (score_max_depth>0)
			{
				if (scene1->points[t].z>score_max_depth)
					continue;
			}

			total_size++;

			int N_nearest = 1;

			std::vector<int> neigh_indices(N_nearest);
			std::vector<float> neigh_sqr_dists(N_nearest);

			int found_neighs = match_search.nearestKSearch(scene->points.at(t), N_nearest, neigh_indices, neigh_sqr_dists);

			//assert(found_neighs == N_nearest);

			float dis = neigh_sqr_dists[0];
			if (dis > max_range)
				continue;

			for (int k = 0; k < N_nearest; ++k)
			{
				pcl::Correspondence corr(t, neigh_indices[k], neigh_sqr_dists[k]);
				_correspondences.push_back(corr);
			}
		}

		_total_size = total_size;

		return _correspondences.size();
	}



	Eigen::Matrix< double, 6, 6 > ComputeInfoMatrix(pcl::Correspondences& cor, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene2)
	{
		// Note:
		// if your transform fit transform*source = target, then you should use information_source. vice versa
		Eigen::Matrix< double, 6, 6 > information_source;
		information_source.setZero();

		Eigen::Matrix< double, 6, 6 > information_target;
		information_target.setZero();


		for (int i = 0; i < cor.size(); i++) {
			pcl::PointXYZRGB source_p = scene1->points[cor.at(i).index_query];
			const float & sx = source_p.x;
			const float & sy = source_p.y;
			const float & sz = source_p.z;
			Eigen::Matrix< double, 3, 6 > A;
			A << 
				1, 0, 0, 0, 2 * sz, -2 * sy,
				0, 1, 0, -2 * sz, 0, 2 * sx,
				0, 0, 1, 2 * sy, -2 * sx, 0;
			information_source += A.transpose() * A;

			pcl::PointXYZRGB target_p = scene2->points[cor.at(i).index_match];
			const float & tx = target_p.x;
			const float & ty = target_p.y;
			const float & tz = target_p.z;
			Eigen::Matrix< double, 3, 6 > AA;
			AA << 1, 0, 0, 0, 2 * tz, -2 * ty,
				0, 1, 0, -2 * tz, 0, 2 * tx,
				0, 0, 1, 2 * ty, -2 * tx, 0;
			information_target += AA.transpose() * AA;
		}

		// default: transform fit transform*source = target
		return information_source;

	}

};