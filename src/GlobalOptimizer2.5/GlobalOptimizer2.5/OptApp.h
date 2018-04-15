#pragma once

#include <unordered_map>
#include <g2o/types/slam3d/se3quat.h>
#include "helper.h"


class COptApp
{
	SeqSaveAndLoad helpSeq_;
	std::unordered_map<Key, MultipleEdge, hash_func, key_equal> multiEdgeContainer_;
public:
	RGBDTrajectory loop_traj_;
	RGBDInformation loop_info_;
	
	RGBDTrajectory pose_traj_;
	RGBDTrajectory loop_remain_traj_;
	RGBDInformation info_remain_;

	// input
	std::string init_seq_file_;
	std::string init_pose_file_;
	std::string loop_log_file_;
	std::string loop_info_file_;
	double weight_;
	int max_iteration_;
	int frame_num_;

	// output
	std::string pose_log_file_;
	std::string loop_remain_log_file_;
	std::string info_remain_log_file_;

public:
	COptApp(int frame_num, std::string init_pose_file, std::string loop_file, std::string loop_info_file, std::string pose_file, std::string loop_remain_log_file, std::string info_remain_file, double weight=50.0)
		:
		frame_num_(frame_num), 
		loop_log_file_(loop_file), 
		loop_info_file_(loop_info_file), 
		pose_log_file_(pose_file), 
		loop_remain_log_file_(loop_remain_log_file), 
		init_pose_file_(init_pose_file), 
		info_remain_log_file_(info_remain_file)
	{
		weight_ = weight;
		max_iteration_ = 200;
	}
	~COptApp(){}
	bool Init();
	void OptimizeSwitchable();
	void OptimizeSlam3d();
	bool BFS();		// judgeLinkAllVertex

private:
	
	Eigen::Matrix4d G2O2Matrix4d( const g2o::SE3Quat& se3 ) {
		Eigen::Matrix4d m = se3.to_homogeneous_matrix(); //_Matrix< 4, 4, double >
		return m;
	}



	g2o::SE3Quat Eigen2G2O( const Eigen::Matrix4d & eigen_mat ) {
		Eigen::Affine3d eigen_transform( eigen_mat );
		Eigen::Quaterniond eigen_quat( eigen_transform.rotation() );
		Eigen::Vector3d translation( eigen_mat( 0, 3 ), eigen_mat( 1, 3 ), eigen_mat( 2, 3 ) );
		g2o::SE3Quat result( eigen_quat, translation );
		return result;
	}
};

