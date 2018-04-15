#pragma once

#include <queue>
#include <map>
#include <vector>

#include <g2o/types/slam3d/se3quat.h>
#include "helper.h"
#include "GraphLoopDetect.h"

struct inx2seq{
	int index;
	int seq;
	inx2seq(int inx, int s) :index(inx), seq(s){}
};

struct inx2seq_cmp{
	bool operator()(inx2seq& i1, inx2seq& i2)
	{
		return i1.index < i2.index;
	}
};


struct key{
	int frame1, frame2;
	key() :frame1(0), frame2(0){}
	key(int id1, int id2) :frame1(id1), frame2(id2){}
};

struct key_comp{
	bool operator () (const key &k1, const key &k2)
	{
		return k1.frame1 < k2.frame1 || (k1.frame1 == k2.frame1 && k1.frame2 < k2.frame2);
	}
};

class ProgressiveOpt{
	int end_flag;
	int Bnum;

	double score_threshold1_;
	double score_threshold2_;
	double error_threshold_;
	double low_score_threshold_;

	SeqSaveAndLoad helpSeq_;
	std::vector<bool> pose_visit;
	std::vector<std::vector<key>> select_loop;
public:
	GraphLoopDetect loopDetect_;

	RGBDTrajectory traj_;
	RGBDInformation info_;

	RGBDTrajectory pose_;

	// input
	std::string traj_file_;
	std::string info_file_;
	int frame_num_;

	// output
	std::string pose_file_;
	std::string fail_file_;
	std::string selected_edge_file_;

	ProgressiveOpt(int frame_num, std::string traj_file, std::string info_file, std::string pose_file, std::string fail_file, std::string selected_edge_file, double low_score_threshold, double score1, double score2, double error_threshold)
		:frame_num_(frame_num), 
		traj_file_(traj_file), 
		info_file_(info_file), 
		pose_file_(pose_file),
		fail_file_(fail_file),
		selected_edge_file_(selected_edge_file),
		error_threshold_(error_threshold)
	{
		end_flag = 0;
		Bnum = 0;
		score_threshold1_ = score1;
		score_threshold2_ = score2;
		low_score_threshold_ = low_score_threshold;
	}

	~ProgressiveOpt(){}

	bool Init();
	void Opt();
	

private:
	std::vector<int> BeamSearch(const double err_threshold, const int beam_width, std::map<key, std::vector<int>, key_comp>& index_map, 
		const std::vector<key>& loop, const std::vector<int>& seq);
	
	void LoopError(const std::vector<std::vector<inx2seq>>& all_loop, int loop_len,
		/*output parameter*/ double& min_error, std::vector<inx2seq>& best_loop);

	//double LoopError(const std::vector<int>& l, std::map<key, std::vector<int>, key_comp>& index_map, const std::vector<key>& loop);

	//double G2oOpt(const std::vector<int>& l, std::map<key, std::vector<int>, key_comp>& index_map, const std::vector<key>& loop, bool final_opt = false);

	void ComputePose(const std::vector<int>& seq);
	void ComputeInitPose(const std::vector<int>& seq);

	/*--------- use some strategies to link all frames after loop search. ---------*/
	// use sequential and score to link.
	void UnionFindSet(std::vector<int>& seq);
	// only use score to link.
	void UnionFindSetNonSequential(std::vector<int>& seq);

	bool judgeLinkAllVertex(std::vector<int>& seq, RGBDTrajectory& loop)
	{
		std::vector<bool> visit_flag(frame_num_, false);
		std::vector<std::vector<int>> graph(frame_num_, std::vector<int>(frame_num_, 0));
		for (int i = 0; i < loop.data_.size();++i)
		{
			if (seq[i] == 1)
			{
				int id1 = loop.data_[i].frame1_;
				int id2 = loop.data_[i].frame2_;

				graph[id1][id2] = 1;
				graph[id2][id1] = 1;
			}
		}
		// BFS
		int visit_num = 0;
		std::queue<int> queue_;
		int start_id = 0;
		queue_.push(start_id);
		visit_flag[start_id] = true;
		visit_num++;
		while (!queue_.empty())
		{
			int inx = queue_.front();
			queue_.pop();

			for (int i = 0; i < frame_num_; ++i)
			{
				if (graph[inx][i] == 1 && visit_flag[i] == false)
				{
					queue_.push(i);
					visit_flag[i] = true;
					visit_num++;
				}
			}
		}

		if (visit_num < frame_num_)
		{
			std::cout << "WARNING: Unconnected graph\n";
			for (int i = 0; i < visit_flag.size(); ++i)
				if (!visit_flag[i])
					std::cout << "Vertex " << i << " unconnect\n";
			return false;
		}
		else
		{
			std::cout << "Connected graph\n";
			return true;
		}
	}

	Eigen::Matrix4d G2O2Matrix4d(const g2o::SE3Quat& se3) {
		Eigen::Matrix4d m = se3.to_homogeneous_matrix(); //_Matrix< 4, 4, double >
		return m;
	}



	g2o::SE3Quat Eigen2G2O(const Eigen::Matrix4d & eigen_mat) {
		Eigen::Affine3d eigen_transform(eigen_mat);
		Eigen::Quaterniond eigen_quat(eigen_transform.rotation());
		Eigen::Vector3d translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
		g2o::SE3Quat result(eigen_quat, translation);
		return result;
	}
};