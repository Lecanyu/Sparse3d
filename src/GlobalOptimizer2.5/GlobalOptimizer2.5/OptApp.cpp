#include "OptApp.h"
#include <queue>

#include <boost/filesystem.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/isometry3d_gradients.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>

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
	// init multiple edge container
	for (int i = 0; i < loop_traj_.data_.size(); ++i)
	{
		int v1 = loop_traj_.data_[i].frame1_;
		int v2 = loop_traj_.data_[i].frame2_;
		int id = i;
		Key key(v1, v2);
		if (multiEdgeContainer_.find(key) == multiEdgeContainer_.end())
		{
			MultipleEdge m;
			m.edge_ids.push_back(id);
			m.weight = loop_info_.data_[id].score_;
			multiEdgeContainer_[key] = m;
		}
		else
		{
			multiEdgeContainer_[key].edge_ids.push_back(id);
			// use maximum weight.
			if (multiEdgeContainer_[key].weight < loop_info_.data_[id].score_)
				multiEdgeContainer_[key].weight = loop_info_.data_[id].score_;
		}
	}

	struct Term1 {
	public:
		// l in term 1
		VertexSwitchLinear * v_;

		// error function in term 2
		/*EdgeSwitchPrior * ep_;*/

		// error function in term 1
		EdgeSE3Switchable * e_;
		// X in term 1
		FramedTransformation * t_;
	};

	g2o::SparseOptimizer* optimizer;
	optimizer = new g2o::SparseOptimizer();
	optimizer->setVerbose(true);
	g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX* solver = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* algo = new g2o::OptimizationAlgorithmLevenberg(solver);
	optimizer->setAlgorithm(algo);

	std::vector< Term1 > term1s;

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

		Term1 edge;
		edge.t_ = &t;

		edge.v_ = new VertexSwitchLinear();
		edge.v_->setId(optimizer->vertices().size());
		edge.v_->setEstimate(1.0);
		optimizer->addVertex(edge.v_);

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
		optimizer->addEdge(edge.e_);		// add term 1 error function into graph 
		term1s.push_back(edge);
	}

	std::vector< EdgeSwitchPrior* > term2s;
	// 
	VertexSwitchLinear temp_v;
	temp_v.setEstimate(-1.0);

	for (auto item : multiEdgeContainer_)
	{
		auto multi_edge = item.second;

		EdgeSwitchPrior* ep_ = new EdgeSwitchPrior(); 
		ep_->vertices()[0] = &temp_v;	ep_->vertices()[1] = &temp_v;	ep_->vertices()[2] = &temp_v;

		for (int i = 0; i < multi_edge.edge_ids.size();++i)
		{
			int edge_id = multi_edge.edge_ids[i];
			VertexSwitchLinear* l = term1s[edge_id].v_;
			ep_->vertices()[i] = l;
		}
		double w = multi_edge.weight;
		ep_->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * w * weight_);
		ep_->setMeasurement(1.0);

		optimizer->addEdge(ep_);		// add term 2 error function into graph 

		term2s.push_back(ep_);
	}

	optimizer->initializeOptimization();
	optimizer->optimize(max_iteration_);

	for (int i = 0; i < pose_traj_.data_.size(); i++) {
		g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3 *>(optimizer->vertex(i));
		pose_traj_.data_[i].transformation_ = G2O2Matrix4d(v->estimateAsSE3Quat());
	}
	pose_traj_.SaveToFile(pose_log_file_);

	loop_remain_traj_.data_.clear();
	info_remain_.data_.clear();
	for (int i = 0; i < term1s.size(); i++) {
		Term1 & edge = term1s[i];
		if (edge.v_->estimate() > 0.25) {
			Eigen::Matrix4d temp = loop_traj_.data_[i].transformation_.inverse();
			loop_remain_traj_.data_.push_back(FramedTransformation(loop_traj_.data_[i].frame1_, loop_traj_.data_[i].frame2_, temp));
			info_remain_.data_.push_back(FramedInformation(loop_info_.data_[i].frame1_, loop_info_.data_[i].frame2_, loop_info_.data_[i].information_, loop_info_.data_[i].score_, loop_info_.data_[i].flag_));
		}
	}
	if (BFS())
	{
		loop_remain_traj_.SaveToFile(loop_remain_log_file_);
		info_remain_.SaveToFile(info_remain_log_file_);
	}

	optimizer->clear();
	delete optimizer;
	g2o::Factory::destroy();
	g2o::OptimizationAlgorithmFactory::destroy();
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


bool COptApp::BFS()
{
	std::vector<bool> visit_flag(frame_num_, false);
	std::vector<std::vector<int>> graph(frame_num_, std::vector<int>(frame_num_, 0));
	for (int i = 0; i < loop_remain_traj_.data_.size(); ++i)
	{
		int id1 = loop_remain_traj_.data_[i].frame1_;
		int id2 = loop_remain_traj_.data_[i].frame2_;

		graph[id1][id2] = 1;
		graph[id2][id1] = 1;
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