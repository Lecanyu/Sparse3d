#include "OptApp.h"

#include <boost/filesystem.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/isometry3d_gradients.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "vertigo3d/vertex_switchLinear.h"
#include "vertigo3d/edge_switchPrior.h"
#include "vertigo3d/edge_se3Switchable.h"

bool COptApp::Init()
{
	namespace fs = boost::filesystem;
	if ( fs::exists( fs::path( loop_log_file_ ) ) ) {
		loop_traj_.LoadFromFile( loop_log_file_ );
		if ( fs::exists( fs::path( loop_info_file_ ) ) ) {
			loop_info_.LoadFromFile( loop_info_file_ );
		}
	}

	for (int i = 0; i < loop_traj_.data_.size(); ++i)
	{
		Eigen::Matrix4d temp = loop_traj_.data_[i].transformation_.inverse();
		loop_traj_.data_[i].transformation_ = temp;
	}

	// all pose init to identity matrix
	if (fs::exists(fs::path(init_pose_file_))) {
		pose_traj_.LoadFromFile(init_pose_file_);
	}
	else
	{
		pose_traj_.data_.clear();
		for (int i = 0; i < frame_num_; ++i)
		{
			Eigen::Matrix4d mat4d_I = Eigen::Matrix4d::Identity();
			pose_traj_.data_.push_back(FramedTransformation(i, i, mat4d_I));
		}
	}
	
	return ( loop_traj_.data_.size() > 0 );
}

void COptApp::OptimizeSwitchable()
{
	struct SwitchableEdge {
	public:
		VertexSwitchLinear * v_;
		EdgeSwitchPrior * ep_;
		EdgeSE3Switchable * e_;
		FramedTransformation * t_;
	};

	g2o::SparseOptimizer* optimizer;
	optimizer = new g2o::SparseOptimizer();
	optimizer->setVerbose(true);
	g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX* solver = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* algo = new g2o::OptimizationAlgorithmLevenberg(solver);
	optimizer->setAlgorithm(algo);

	std::vector< SwitchableEdge > switch_edge;

	Eigen::Matrix< double, 6, 6 > default_information;
	default_information = Eigen::Matrix< double, 6, 6 >::Identity();

	for (int i = 0; i < frame_num_; i++) {
		g2o::VertexSE3 * v = new g2o::VertexSE3();
		v->setId(i);
		v->setEstimate(Eigen2G2O(pose_traj_.data_[i].transformation_));
		if (i == 0) {
			v->setFixed(true);
		}
		optimizer->addVertex(v);
	}

	for (int i = 0; i < (int)loop_traj_.data_.size(); i++) {
		FramedTransformation & t = loop_traj_.data_[i];

		SwitchableEdge edge;
		edge.t_ = &t;

		edge.v_ = new VertexSwitchLinear();
		edge.v_->setId(optimizer->vertices().size());
		edge.v_->setEstimate(1.0);
		optimizer->addVertex(edge.v_);

		edge.ep_ = new EdgeSwitchPrior();
		edge.ep_->vertices()[0] = edge.v_;
		edge.ep_->setMeasurement(1.0);
		edge.ep_->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * loop_info_.data_[i].score_ * weight_);
		optimizer->addEdge(edge.ep_);

		edge.e_ = new EdgeSE3Switchable();
		edge.e_->vertices()[0] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex(t.frame1_));
		edge.e_->vertices()[1] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex(t.frame2_));
		edge.e_->vertices()[2] = edge.v_;
		edge.e_->setMeasurement(g2o::internal::fromSE3Quat(Eigen2G2O(t.transformation_)));
		if (loop_info_.data_.size() > 0) {
			edge.e_->setInformation(loop_info_.data_[i].information_);
		}
		else {
			edge.e_->setInformation(default_information);
		}
		optimizer->addEdge(edge.e_);
		switch_edge.push_back(edge);
	}

	optimizer->initializeOptimization();
	optimizer->optimize(max_iteration_);

	for (int i = 0; i < (int)pose_traj_.data_.size(); i++) {
		g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3 *>(optimizer->vertex(i));
		pose_traj_.data_[i].transformation_ = G2O2Matrix4d(v->estimateAsSE3Quat());
	}
	pose_traj_.SaveToFile(pose_log_file_);

	loop_remain_traj_.data_.clear();
	info_remain_.data_.clear();
	for (int i = 0; i < (int)switch_edge.size(); i++) {
		SwitchableEdge & edge = switch_edge[i];
		if (edge.v_->estimate() > 0.25) {
			Eigen::Matrix4d temp = loop_traj_.data_[i].transformation_.inverse();
			loop_remain_traj_.data_.push_back(FramedTransformation(loop_traj_.data_[i].frame1_, loop_traj_.data_[i].frame2_, temp));
			info_remain_.data_.push_back(FramedInformation(loop_info_.data_[i].frame1_, loop_info_.data_[i].frame2_, loop_info_.data_[i].information_, loop_info_.data_[i].score_));
		}
	}
	loop_remain_traj_.SaveToFile(loop_remain_log_file_);
	info_remain_.SaveToFile(info_remain_log_file_);

}


void COptApp::OptimizeSlam3d()
{
	g2o::SparseOptimizer* optimizer;
	optimizer = new g2o::SparseOptimizer();
	optimizer->setVerbose(true);
	g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX* solver = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* algo = new g2o::OptimizationAlgorithmLevenberg(solver);
	optimizer->setAlgorithm(algo);

	Eigen::Matrix< double, 6, 6 > default_information;
	default_information = Eigen::Matrix< double, 6, 6 >::Identity();

	for (int i = 0; i < frame_num_; i++) {
		g2o::VertexSE3 * v = new g2o::VertexSE3();
		v->setId(i);
		v->setEstimate(Eigen2G2O(pose_traj_.data_[i].transformation_));
		if (i == 0) {
			v->setFixed(true);
		}
		optimizer->addVertex(v);
	}

	std::vector<double> init_error_arr;
	for (int i = 0; i < (int)loop_traj_.data_.size(); i++) {

		FramedTransformation & t = loop_traj_.data_[i];

		g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3();
		g2o_edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex(t.frame1_));
		g2o_edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex(t.frame2_));
		g2o_edge->setMeasurement(g2o::internal::fromSE3Quat(Eigen2G2O(t.transformation_)));

		auto f = g2o::internal::toVectorMQT(g2o::internal::fromSE3Quat(Eigen2G2O(t.transformation_)));
		double err = f.transpose() * loop_info_.data_[i].information_ * f;
		init_error_arr.push_back(err);

		if (loop_info_.data_.size() > 0) {
			g2o_edge->setInformation(loop_info_.data_[i].information_);
		}
		else {
			g2o_edge->setInformation(default_information);
		}
		optimizer->addEdge(g2o_edge);
	}
	std::sort(init_error_arr.begin(), init_error_arr.end(), std::less<double>());
	std::cout << "min: " <<init_error_arr[0] << "max: " << init_error_arr[init_error_arr.size() - 1] << "\n";

	optimizer->initializeOptimization();
	optimizer->optimize(max_iteration_);

	for (int i = 0; i < (int)pose_traj_.data_.size(); i++) {
		g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3 *>(optimizer->vertex(i));
		pose_traj_.data_[i].transformation_ = G2O2Matrix4d(v->estimateAsSE3Quat());
	}
	pose_traj_.SaveToFile(pose_log_file_);
}