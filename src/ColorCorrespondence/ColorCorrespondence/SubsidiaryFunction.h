#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <fstream>

struct CameraParam {
public:
	double fx_, fy_, cx_, cy_;
	double k1_, k2_, k3_, p1_, p2_;
	double depth_ratio_;
	double downsample_leaf;

	int img_width_;
	int img_height_;

	float integration_trunc_;
	CameraParam() :
		fx_(525.0f), fy_(525.0f), cx_(319.5f), cy_(239.5f),
		k1_(0.0), k2_(0.0), k3_(0.0), p1_(0.0), p2_(0.0),
		depth_ratio_(1000.0),
		downsample_leaf(0.05),
		img_width_(640), img_height_(480),
		integration_trunc_(4.0)
	{}

	void LoadFromFile(std::string filename) {
		FILE * f = fopen(filename.c_str(), "r");
		if (f != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%lf", &fx_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &fy_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &cx_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &cy_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &k1_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &k2_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &k3_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &p1_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &p2_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &depth_ratio_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &downsample_leaf);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%d", &img_width_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%d", &img_height_);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf", &integration_trunc_);
				}
			}
			printf("Camera model set to (fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f, k1: %.2f, k2: %.2f, k3: %.2f, p1: %.2f, p2: %.2f, depth_ratio_: %.2f, downsample_leaf: %.2f, img_width_: %d, img_height_: %d, integration_trunc: %.2f)\n",
				fx_, fy_, cx_, cy_, k1_, k2_, k3_, p1_, p2_, depth_ratio_, downsample_leaf, img_width_, img_height_, integration_trunc_);
			fclose(f);
		}
	}
};


void ConvertRGBCloud2XYZCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz);

void ConvertXYZCloud2RGBCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr xyz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb);

bool UVD2XYZ(int u, int v, unsigned short d, CameraParam& _camera, double& x, double& y, double& z);

pcl::PointXYZRGB  SearchNearestValidPoint(int u, int v, cv::Mat& depth, CameraParam& camera);


// correspondence
struct quad
{
	int image1_pixel_x, image1_pixel_y;
	int image2_pixel_x, image2_pixel_y;
};

struct CorrespondencePixelData
{
	int imageid1_, imageid2_;
	std::vector<quad> pixel_correspondence_;
};

struct CorrespondencePixel
{
	std::vector<CorrespondencePixelData> data_;
	void LoadFromFile(std::string filename) {
		std::ifstream input(filename);
		if (!input.good()){
			std::cerr << "Can't open file " << filename << "!" << std::endl;
			exit(-1);
		}

		while (input.good()){
			std::string line;
			std::getline(input, line);
			if (line.empty()) continue;
			std::stringstream ss(line);     //×Ö·û´®×ª×Ö·ûÁ÷
			std::string keyword;
			ss >> keyword;
			if (keyword == "#")
				continue;
			if (keyword == "image_id")
			{
				int id1, id2;
				ss >> id1 >> id2;

				CorrespondencePixelData data;
				data.imageid1_ = id1;
				data.imageid2_ = id2;

				data_.push_back(data);

				continue;
			}

			int a, b, c, d;
			a = atoi(keyword.c_str());
			ss >> b >> c >> d;

			quad q;
			q.image1_pixel_x = a;	q.image1_pixel_y = b;
			q.image2_pixel_x = c;	q.image2_pixel_y = d;
			data_[data_.size() - 1].pixel_correspondence_.push_back(q);
		}

	}
};

struct FramedTransformation {
	int frame1_;
	int frame2_;
	Eigen::Matrix4d transformation_;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	FramedTransformation()
		:frame1_(-1), frame2_(-1), transformation_(Eigen::Matrix4d::Identity())
	{}
	FramedTransformation(int frame1,int frame2, const Eigen::Matrix4d& t)
		: frame1_(frame1), frame2_(frame2), transformation_(t)
	{}
};



struct RGBDTrajectory {
	std::vector< FramedTransformation, Eigen::aligned_allocator<FramedTransformation>> data_;

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

	void SaveToFileAppend(std::string filename)
	{
		FILE * f = fopen(filename.c_str(), "a+");
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

	void SaveToSPFile(std::string filename) {
		FILE * f = fopen(filename.c_str(), "w");
		for (int i = 0; i < (int)data_.size(); i++) {
			if (data_[i].frame1_==-1)
				continue;
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
	InformationMatrix information_;

	double score_;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	FramedInformation() 
		:frame1_(-1), frame2_(-1), score_(-1.0)
	{
		information_ <<
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1;
	}
	FramedInformation(int frame1, int frame2, const InformationMatrix& t, double score)
		: frame1_(frame1), frame2_(frame2), information_(t), score_(score)
	{}
};


struct RGBDInformation {
	std::vector< FramedInformation, Eigen::aligned_allocator<FramedInformation> > data_;

	void LoadFromFile(std::string filename) {
		data_.clear();
		int frame1, frame2;
		double score;
		InformationMatrix info;
		FILE * f = fopen(filename.c_str(), "r");
		if (f != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d %d %lf", &frame1, &frame2,&score);
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
					data_.push_back(FramedInformation(frame1, frame2, info, score));
				}
			}
			fclose(f);
		}
	}
	void SaveToFile(std::string filename) {
		FILE * f = fopen(filename.c_str(), "w");
		for (int i = 0; i < (int)data_.size(); i++) {
			InformationMatrix & info = data_[i].information_;
			fprintf(f, "%d\t%d\t%.8f\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_);
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
		}
		fclose(f);
	}
	void SaveToFileAppend(std::string filename) {
		FILE * f = fopen(filename.c_str(), "a+");
		for (int i = 0; i < (int)data_.size(); i++) {
			InformationMatrix & info = data_[i].information_;
			fprintf(f, "%d\t%d\t%.8f\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_);
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
		}
		fclose(f);
	}
	void SaveToSPFile(std::string filename) {
		FILE * f = fopen(filename.c_str(), "w");
		for (int i = 0; i < (int)data_.size(); i++) {
			if (data_[i].frame1_==-1)
				continue;
			InformationMatrix & info = data_[i].information_;
			fprintf(f, "%d\t%d\t%.8f\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_);
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
