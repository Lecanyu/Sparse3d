#pragma once

#include <vector>
#include <fstream>
#include <Eigen/Core>

struct FramedTransformation {
	int frame1_;
	int frame2_;
	Eigen::Matrix4d transformation_;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	FramedTransformation(int frame1, int frame2, const Eigen::Matrix4d& t)
		: frame1_(frame1), frame2_(frame2), transformation_(t)
	{}
};



struct RGBDTrajectory {
	std::vector< FramedTransformation, Eigen::aligned_allocator<FramedTransformation>> data_;

	void LoadFromSPFile(std::string filename) {
		data_.clear();
		int frame1, frame2;
		int dd;
		Eigen::Matrix4d trans;
		FILE * f = fopen(filename.c_str(), "r");
		if (f != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d %d %d", &frame1, &frame2, &dd);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));

					data_.push_back(FramedTransformation(frame1, frame2, trans));
				}
			}
			fclose(f);
		}
	}


	void LoadFromFile(std::string filename) {
		data_.clear();
		int frame1, frame2;
		Eigen::Matrix4d trans;
		FILE * f = fopen(filename.c_str(), "r");
		if (f != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d %d", &frame1, &frame2);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));

					data_.push_back(FramedTransformation(frame1, frame2, trans));
				}
			}
			fclose(f);
		}
	}
	void SaveToFile(std::string filename) {
		FILE * f = fopen(filename.c_str(), "w");
		for (int i = 0; i < (int)data_.size(); i++) {
			Eigen::Matrix4d & trans = data_[i].transformation_;
			fprintf(f, "%d\t%d\n", data_[i].frame1_, data_[i].frame2_);
			fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
			fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
			fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
			fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
		}
		fclose(f);
	}
};


typedef Eigen::Matrix< double, 6, 6, Eigen::RowMajor > InformationMatrix;
struct FramedInformation {
	int frame1_;
	int frame2_;
	double score_;
	InformationMatrix information_;

	int flag;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	FramedInformation(int frame1, int frame2, const InformationMatrix& t)
		: frame1_(frame1), frame2_(frame2), information_(t), score_(0.0)
	{}
	FramedInformation(int frame1, int frame2, const InformationMatrix& t, double score,int f)
		: frame1_(frame1), frame2_(frame2), information_(t), score_(score), flag(f)
	{}
};


struct RGBDInformation {
	std::vector< FramedInformation, Eigen::aligned_allocator<FramedInformation> > data_;

	void LoadFromFile(std::string filename) {
		data_.clear();
		int flag;
		int frame1, frame2;
		double score;
		InformationMatrix info;
		FILE * f = fopen(filename.c_str(), "r");
		if (f != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d %d %lf %d", &frame1, &frame2, &score, &flag);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(0, 0), &info(0, 1), &info(0, 2), &info(0, 3), &info(0, 4), &info(0, 5));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(1, 0), &info(1, 1), &info(1, 2), &info(1, 3), &info(1, 4), &info(1, 5));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(2, 0), &info(2, 1), &info(2, 2), &info(2, 3), &info(2, 4), &info(2, 5));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(3, 0), &info(3, 1), &info(3, 2), &info(3, 3), &info(3, 4), &info(3, 5));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(4, 0), &info(4, 1), &info(4, 2), &info(4, 3), &info(4, 4), &info(4, 5));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(5, 0), &info(5, 1), &info(5, 2), &info(5, 3), &info(5, 4), &info(5, 5));
					data_.push_back(FramedInformation(frame1, frame2, info, score, flag));
				}
			}
			fclose(f);
		}
	}

	void SaveToFile(std::string filename) {
		FILE * f = fopen(filename.c_str(), "w");
		for (int i = 0; i < (int)data_.size(); i++) {
			InformationMatrix & info = data_[i].information_;
			fprintf(f, "%d\t%d\t%.8f\t%d\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_, data_[i].flag);
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
		}
		fclose(f);
	}
};




struct SeqSaveAndLoad{
	template<typename T>
	bool Load(std::string filename, std::vector<T>& seq)
	{
		std::ifstream input(filename);
		if (!input.good())
			return false;
		while (input.good()){
			std::string str;
			input >> str;
			if (str != ""&&str != "\n")
			{
				std::stringstream ss(str);
				T l;
				ss >> l;
				seq.push_back(l);
			}
		}

		return true;
	}

	template<typename T>
	bool Save(std::string filename, std::vector<T>& seq)
	{
		std::ofstream output(filename);
		for (int i = 0; i < seq.size(); ++i)
		{
			if (i < seq.size() - 1)
				output << seq[i] << "\n";
			else
				output << seq[i];
		}

		return true;
	}

	template<typename T>
	bool Save(std::string filename, std::vector<T>& seq, RGBDTrajectory& loop)
	{
		std::ofstream output(filename);
		for (int i = 0; i < seq.size(); ++i)
		{
			if (i < seq.size() - 1)
				output << loop.data_[i].frame1_ << "---" << loop.data_[i].frame2_ << ":  " << seq[i] << "\n";
			else
				output << loop.data_[i].frame1_ << "---" << loop.data_[i].frame2_ << ":  " << seq[i];
		}
		return true;
	}
};



#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/isometry3d_gradients.h>

struct CheckAccuracy
{
	std::vector< FramedTransformation, Eigen::aligned_allocator<FramedTransformation> > groundtruth_pose;
	std::vector< FramedTransformation, Eigen::aligned_allocator<FramedTransformation> > traj_data;

	void init(std::string& groundtruth_pose_file, std::string& traj_file)
	{
		int frame1, frame2;
		Eigen::Matrix4d trans;

		// groundtruth pose
		groundtruth_pose.clear();
		FILE * f1 = fopen(groundtruth_pose_file.c_str(), "r");
		if (f1 != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f1) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d %d", &frame1, &frame2);
					fgets(buffer, 1024, f1);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
					fgets(buffer, 1024, f1);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
					fgets(buffer, 1024, f1);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
					fgets(buffer, 1024, f1);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));

					groundtruth_pose.push_back(FramedTransformation(frame1, frame2, trans));
				}
			}
			fclose(f1);
		}

		// traj
		traj_data.clear();
		FILE * f2 = fopen(traj_file.c_str(), "r");
		if (f2 != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f2) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d %d", &frame1, &frame2);
					fgets(buffer, 1024, f2);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
					fgets(buffer, 1024, f2);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
					fgets(buffer, 1024, f2);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
					fgets(buffer, 1024, f2);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));

					traj_data.push_back(FramedTransformation(frame1, frame2, trans));
				}
			}
			fclose(f2);
		}
	}

	// check init_pose accuracy
	void init(std::string& groundtruth_pose_file, std::string& traj_file, std::string& init_seq)
	{
		int frame1, frame2;
		Eigen::Matrix4d trans;

		// groundtruth pose
		groundtruth_pose.clear();
		FILE * f1 = fopen(groundtruth_pose_file.c_str(), "r");
		if (f1 != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f1) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d %d", &frame1, &frame2);
					fgets(buffer, 1024, f1);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
					fgets(buffer, 1024, f1);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
					fgets(buffer, 1024, f1);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
					fgets(buffer, 1024, f1);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));

					groundtruth_pose.push_back(FramedTransformation(frame1, frame2, trans));
				}
			}
			fclose(f1);
		}

		// init seq
		std::vector<double> seq;
		std::ifstream input(init_seq);
		if (!input.good())
		{
			std::cout << "ERROR: INIT_SEQ FILE INVALID\n";
			exit(-1);
		}

		while (input.good()){
			std::string str;
			input >> str;
			if (str != ""&&str != "\n")
			{
				std::stringstream ss(str);
				double l;
				ss >> l;
				seq.push_back(l);
			}
		}

		// traj
		std::vector< FramedTransformation, Eigen::aligned_allocator<FramedTransformation> > temp;
		temp.clear();
		FILE * f2 = fopen(traj_file.c_str(), "r");
		if (f2 != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f2) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d %d", &frame1, &frame2);
					fgets(buffer, 1024, f2);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
					fgets(buffer, 1024, f2);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
					fgets(buffer, 1024, f2);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
					fgets(buffer, 1024, f2);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));

					temp.push_back(FramedTransformation(frame1, frame2, trans));
				}
			}
			fclose(f2);
		}

		traj_data.clear();
		for (int i = 0; i < seq.size(); ++i)
		{
			if (seq[i]>0.5)
			{
				traj_data.push_back(temp[i]);
			}
		}
	}


	double TwoFrameError(int id1, int id2, Eigen::Matrix4d& traj)
	{
		assert(id1 < id2 &&"ERROR CALL, CHECK DATA\n");
		Eigen::Matrix4d err_mat = groundtruth_pose[id2].transformation_*traj*groundtruth_pose[id1].transformation_.inverse();

		Eigen::Isometry3d delta(err_mat);

		auto err = g2o::internal::toVectorMQT(delta);
		return sqrt(err.transpose()*err);
	}


	// Compute accuracy
	void CountTotalAcc()
	{
		double threshold = 0.62;
		int err_edge = 0;

		double max_val = -999.0;
		for (int i = 0; i < traj_data.size(); ++i)
		{
			if (traj_data[i].frame1_ == 33 && traj_data[i].frame2_ == 52)
			{
				std::cout << "";
			}
			double err = TwoFrameError(traj_data[i].frame1_, traj_data[i].frame2_, traj_data[i].transformation_);
			if (err > threshold)
				err_edge++;
			if (max_val < err)
				max_val = err;
		}

		double result = (double)err_edge / traj_data.size();
		std::cout << "error rate: " << result << "\n";
	}
};