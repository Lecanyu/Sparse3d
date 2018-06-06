#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <fstream>
#include <algorithm> 

#include <armadillo>  

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/common/transforms.h>

#include <Eigen/Eigen>


class GraphMatching
{

public:
	int _nrow;
	int _ncol;
	arma::sp_fmat _m;
	pcl::PointCloud<pcl::PointXYZRGB>& _keypoints1;
	pcl::PointCloud<pcl::PointXYZRGB>& _keypoints2;

	// final transformation between two frames
	Eigen::Matrix4f		_transformation;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	
	
	GraphMatching(pcl::PointCloud<pcl::PointXYZRGB>& keypoints1, pcl::PointCloud<pcl::PointXYZRGB>& keypoints2, const pcl::Correspondences& correspondence);
	// refine correspondences
	pcl::CorrespondencesPtr		ComputeCorrespondenceByEigenVec(int best_num = 4);
	// give n pairs correspondences, compute rigid transformation
	Eigen::Matrix4f		ComputeRigid(pcl::CorrespondencesConstPtr correspondence, const pcl::PointCloud<pcl::PointXYZRGB>& keypoints1, const pcl::PointCloud<pcl::PointXYZRGB>& keypoints2);
	
	
private:
	// judge n points in the same plane or not
	bool judge_pointn_plane(std::vector<pcl::PointXYZ>& arr)
	{
		Eigen::Vector3f v1, v2, v3;
		v1 <<
			arr[1].x - arr[0].x, arr[1].y - arr[0].y, arr[1].z - arr[0].z;
		v2 <<
			arr[2].x - arr[0].x, arr[2].y - arr[0].y, arr[2].z - arr[0].z;

		Eigen::Vector3f n;
		n = v1.cross(v2);
		n.normalize();

		for (int i = 3; i < arr.size(); ++i)
		{
			v3 <<
				arr[i].x - arr[0].x, arr[i].y - arr[0].y, arr[i].z - arr[0].z;
			float dis = v3.dot(n);
			dis = fabs(dis);

			float the_same_plane_threshold = 0.11f;

			if (dis > the_same_plane_threshold)
			{
				return false;
			}
		}
		return true;
	}
};

