#include "ProgressiveOpt.h"
#include "Verbose.h"

#include <boost/filesystem.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/isometry3d_gradients.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

bool ProgressiveOpt::Init()
{
	namespace fs = boost::filesystem;
	if (fs::exists(fs::path(traj_file_))) {
		traj_.LoadFromFile(traj_file_);
		if (fs::exists(fs::path(info_file_))) {
			info_.LoadFromFile(info_file_);
		}
	}

	for (int i = 0; i < traj_.data_.size(); ++i)
	{
		Eigen::Matrix4d temp = traj_.data_[i].transformation_.inverse();
		traj_.data_[i].transformation_ = temp;
	}

	// all pose init to identity matrix
	for (int i = 0; i < frame_num_; ++i)
	{
		Eigen::Matrix4d mat4d_I = Eigen::Matrix4d::Identity();
		pose_.data_.push_back(FramedTransformation(i, i, mat4d_I));
	}

	//pose_.SaveToFile("C:/Users/range/Desktop/multiedge/init_pose.txt");

	pose_visit.resize(frame_num_, false);
	return (traj_.data_.size() > 0);
}


void recur_seq(const std::vector<int>& frag_len, const std::vector<int>& inx, int level, std::vector<inx2seq> item, std::vector<std::vector<inx2seq>>& result, const int beam_width)
{
	if (result.size() <= beam_width)
	{
		if (level == frag_len.size())
		{
			result.push_back(item);
		}
		else
		{
			for (int i = 0; i < frag_len[level]; ++i)
			{
				std::vector<inx2seq> item_copy = item;
				int inx_index = 0;
				for (int t = 0; t < level; ++t)
					inx_index += frag_len[t];

				for (int j = 0; j < frag_len[level]; ++j)
				{
					if (j == i){
						inx2seq is(inx[inx_index + j], 1);
						item_copy.push_back(is);
					}
					else
					{
						inx2seq is(inx[inx_index + j], 0);
						item_copy.push_back(is);
					}

				}
				recur_seq(frag_len, inx, level + 1, item_copy, result, beam_width);
			}
		}
	}
}

void ProgressiveOpt::LoopError(const std::vector<std::vector<inx2seq>>& all_loop, int loop_len, 
	/*output parameter*/ double& min_error, std::vector<inx2seq>& best_loop)
{
	min_error = 99999.0;
	best_loop.clear();

	for (int i = 0; i < all_loop.size();++i)
	{
		const std::vector<inx2seq>& one_loop = all_loop[i];

		RGBDTrajectory t_arr;
		RGBDInformation i_arr;
		Eigen::Matrix4d err = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d final_mat = Eigen::Matrix4d::Identity();
		for (int k = 0; k < one_loop.size(); k++) {
			if (one_loop[k].seq < 1)
				continue;
			FramedTransformation & t = traj_.data_[one_loop[k].index];
			t_arr.data_.push_back(t);

			FramedInformation & inf = info_.data_[one_loop[k].index];
			i_arr.data_.push_back(inf);
		}

		assert(t_arr.data_[0].frame1_ == t_arr.data_[1].frame1_);

		double rotation_deg = -9999.0;
		int low_score_edge = 0;
		int geometric_num = 0;
		for (int i = 0; i < t_arr.data_.size(); ++i)
		{
			if (i == 1)
			{
				final_mat = t_arr.data_[i].transformation_;
				if (i_arr.data_[i].flag == 0)
					geometric_num++;
				if (i_arr.data_[i].flag == 0&&i_arr.data_[i].score_<low_score_threshold_)
					low_score_edge++;

				Eigen::Matrix3d mat3;
				mat3 <<
					t_arr.data_[i].transformation_(0, 0), t_arr.data_[i].transformation_(0, 1), t_arr.data_[i].transformation_(0, 2),
					t_arr.data_[i].transformation_(1, 0), t_arr.data_[i].transformation_(1, 1), t_arr.data_[i].transformation_(1, 2),
					t_arr.data_[i].transformation_(2, 0), t_arr.data_[i].transformation_(2, 1), t_arr.data_[i].transformation_(2, 2);
				Eigen::Quaterniond q(mat3);
				double deg = acos(q.w()) * 180 / 3.1415;
				deg *= 2;
				if (rotation_deg<deg)
					rotation_deg = deg;
			}
			else
			{
				err = err*t_arr.data_[i].transformation_;
				if (i_arr.data_[i].flag == 0)
					geometric_num++;
				if (i_arr.data_[i].flag == 0 && i_arr.data_[i].score_ < low_score_threshold_)
					low_score_edge++;

				Eigen::Matrix3d mat3;
				mat3 <<
					t_arr.data_[i].transformation_(0, 0), t_arr.data_[i].transformation_(0, 1), t_arr.data_[i].transformation_(0, 2),
					t_arr.data_[i].transformation_(1, 0), t_arr.data_[i].transformation_(1, 1), t_arr.data_[i].transformation_(1, 2),
					t_arr.data_[i].transformation_(2, 0), t_arr.data_[i].transformation_(2, 1), t_arr.data_[i].transformation_(2, 2);
				Eigen::Quaterniond q(mat3);
				double deg = acos(q.w())*180/3.1415;
				deg *= 2;
				if (rotation_deg < deg)
					rotation_deg = deg;
			}
		}

		err = err*final_mat.inverse();
		Eigen::Isometry3d delta(err);
		auto error = g2o::internal::toVectorMQT(delta);
		double translation_err = error(0)*error(0) + error(1)*error(1) + error(2)*error(2);
		double rotation_err = 4 * (error(3)*error(3) + error(4)*error(4) + error(5)*error(5));
		double e = translation_err + rotation_err;

		if (loop_len==3)
		{
			if (geometric_num >=2)
				e += 0.005;
		}
		if (low_score_edge>0)
		{
			e += 999.0;
		}
		if (rotation_deg>80.0)			// if rotation degree too big discard it.
		{
			e += 999.0;
		}
		if (e<min_error)
		{
			min_error = e;
			best_loop = one_loop;
		}
	}
}


std::vector<int> ProgressiveOpt::BeamSearch(const double err_threshold, const int beam_width, std::map<key, std::vector<int>, key_comp>& index_map, const std::vector<key>& loop, const std::vector<int>& seq)
{
	for (int i = 0; i < loop.size(); ++i)
		if (loop[i].frame1 == frame_num_ - 1 || loop[i].frame2 == frame_num_ - 1)
			end_flag++;
	Bnum++;
#ifdef Verbose
	std::cout << "-----------------Loop" << Bnum << "----------------\n";
#endif
	
	int unfix_edge = 0;

	std::vector<inx2seq> fix_part;
	int all_seq_len = 1;
	// init one loop edge
	for (int i = 0; i < loop.size(); ++i)
	{
		std::vector<int>& vec = index_map[loop[i]];
		if (seq[vec[0]] == -1)		// not fix
		{
			all_seq_len *= vec.size();
			unfix_edge++;
		}
		else
		{
			for (int j = 0; j < vec.size(); ++j)
			{
				inx2seq is(vec[j], seq[vec[j]]);
				fix_part.push_back(is);
			}
		}
	}

	if (unfix_edge > 50)
		return seq;

	std::vector<int> frag_len;
	std::vector<int> inx;
	for (int i = 0; i < loop.size(); ++i)
	{
		std::vector<int>& vec = index_map[loop[i]];
		if (seq[vec[0]] == -1)
		{
			inx.insert(inx.end(), vec.begin(), vec.end());
			frag_len.push_back(vec.size());
		}
	}
	//std::sort(inx.begin(), inx.end(), std::less<int>());

	std::vector<inx2seq> item;
	std::vector<std::vector<inx2seq>> result;
	recur_seq(frag_len, inx, 0, item, result, beam_width);				// recursive compute all possible solution

	for (int i = 0; i < result.size();++i)
	{
		std::vector<inx2seq>& one_loop = result[i];
		one_loop.insert(one_loop.end(), fix_part.begin(), fix_part.end());
		std::sort(one_loop.begin(), one_loop.end(), inx2seq_cmp());				// one solution
	}

	// compute minimum error
	double min_error;
	std::vector<inx2seq> best_loop;
	int loop_len = loop.size();
	LoopError(result, loop_len, min_error, best_loop);

	// update
	if (min_error < err_threshold)
	{
		std::vector<int> best_l = seq;
		for (int i = 0; i < best_loop.size(); ++i)
			best_l[best_loop[i].index] = best_loop[i].seq;

		for (int i = 0; i < best_l.size(); ++i)
		{
			if (best_l[i] == 1)
			{
				int v1 = traj_.data_[i].frame1_;
				int v2 = traj_.data_[i].frame2_;
				pose_visit[v1] = true;
				pose_visit[v2] = true;
			}
		}
		select_loop.push_back(loop);
#ifdef Verbose
		std::cout << "Successful!\n";
		std::cout << "loop Error: " << min_error << ";  Threshold:" << err_threshold << "\n";
#endif
		return best_l;
	}
	else
	{
#ifdef Verbose
		std::cout << "Fail!\n";
		std::cout << "loop Error: " << min_error << ";  Threshold:" << err_threshold << "\n";
#endif
		return seq;
	}
}


void ProgressiveOpt::Opt()
{
	std::map<key, std::vector<int>, key_comp> index_map;
	for (int i = 0; i < traj_.data_.size(); ++i)
	{
		int frame1 = traj_.data_[i].frame1_;
		int frame2 = traj_.data_[i].frame2_;
		index_map[key(frame1, frame2)].push_back(i);
	}
	std::vector<int> seq(traj_.data_.size(), -1);

	if (loopDetect_.Init(traj_file_, frame_num_))
	{
		struct state{
			bool in_stack;
			int level;
			int start_vertex;
			state() :in_stack(false), level(-1), start_vertex(0){}
			state(bool ins, int le, int start) :in_stack(ins), level(le), start_vertex(start){}
		};
		std::vector<state> stack_vertex(frame_num_);

		// DFS search all loop
		std::stack<int> DFS;
		DFS.push(0);
		stack_vertex[0] = state(true, 0, 0);

		int no_change_num = 0;
		std::vector<bool> last_visit(frame_num_, false);
		while (!DFS.empty())
		{
			int item = DFS.top();
			bool add_stack = false;
			for (int i = stack_vertex[item].start_vertex; i < frame_num_; ++i)
			{
				if (loopDetect_.graph[item][i] == 1 && stack_vertex[i].in_stack == false)
				{
					DFS.push(i);

					int last_level = stack_vertex[item].level;
					stack_vertex[i] = state(true, last_level + 1, 0);
					add_stack = true;

					stack_vertex[item].start_vertex = i + 1;
					break;
				}
				if (loopDetect_.graph[item][i] == 1 && stack_vertex[i].in_stack == true)		// one loop
				{
					assert(stack_vertex[item].level != -1 && stack_vertex[i].level != -1);
					int loop_len = stack_vertex[item].level - stack_vertex[i].level;
					if (loop_len >= 2)
					{
						std::stack<int> temp_stack = DFS;
						std::vector<int> loop;
						while (temp_stack.top() != i)
						{
							loop.push_back(temp_stack.top());
							temp_stack.pop();
						}
						loop.push_back(i);
						loop.push_back(item);

						std::vector<key> one_loop;
						for (int j = 0; j < loop.size() - 1; ++j)
						{
							int v1 = loop[j];
							int v2 = loop[j + 1];
							if (v1 > v2)
							{
								int temp = v1;
								v1 = v2;
								v2 = temp;
							}
							key k(v1, v2);
							one_loop.push_back(k);
						}
						
						double err_threshold;
						if (one_loop.size() < 10)
							err_threshold = sqrt(one_loop.size())*error_threshold_;
						else
							err_threshold = one_loop.size() * error_threshold_;
						
						seq = BeamSearch(err_threshold, 1024, index_map, one_loop, seq);
					}
				}

				// end flag
				bool all_visit = true;
				for (int i = 0; i < pose_visit.size(); ++i)
					if (pose_visit[i] == false)
						all_visit = false;
				if (all_visit || end_flag >= 10)
					break;
			}
			if (!add_stack)
			{
				DFS.pop();
				stack_vertex[item].in_stack = false;
				stack_vertex[item].level = -1;
				stack_vertex[item].start_vertex = 0;
			}

			// end flag
			bool all_visit = true;
			bool visit_change = false;
			for (int i = 0; i < pose_visit.size(); ++i)
			{
				if (pose_visit[i] == false)
					all_visit = false;
				if (pose_visit[i] != last_visit[i])
					visit_change = true;
			}
			if (all_visit || end_flag >= 10)
				break;

			if (visit_change)
			{
				no_change_num = 0;
				last_visit = pose_visit;
			}
			else
			{
				no_change_num++;
			}
			if (no_change_num>150)
				break;
		}
	}
	// connect all vertex
	// Strategy 1: use sequential+score. Better result 
	//UnionFindSet(seq);
	// Strategy 2: only use score. Useful for unordered dataset.
	UnionFindSetNonSequential(seq);

	if (judgeLinkAllVertex(seq, traj_))
	{
		ComputeInitPose(seq);
		//pose_.SaveToFile(init_pose_file_);
		ComputePose(seq);
		pose_.SaveToFile(pose_file_);
	}
}


void ProgressiveOpt::ComputeInitPose(const std::vector<int>& seq)
{
	std::vector<int> init_seq(seq.size(), 0);
	std::vector<bool> vertex_visit(frame_num_, false);

	struct edge{
		int val;
		int id;
		edge() :val(0), id(-1){}
		edge(int v, int i) :val(v), id(i){}
	};
	std::vector<std::vector<edge>> graph(frame_num_, std::vector<edge>(frame_num_));
	for (int i = 0; i < seq.size();++i)
	{
		if (seq[i]==1)
		{
			int v1 = traj_.data_[i].frame1_;
			int v2 = traj_.data_[i].frame2_;
			edge e(1, i);
			graph[v1][v2] = e;
			graph[v2][v1] = e;
		}
	}

	//DFS
	struct state{
		bool in_stack;
		int level;
		int start_vertex;

		Eigen::Matrix4d transformation_;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		state() :in_stack(false), level(-1), start_vertex(0), transformation_(Eigen::Matrix4d::Identity()){}
		state(bool ins, int le, int start, Eigen::Matrix4d& t) :in_stack(ins), level(le), start_vertex(start),transformation_(t){}
	};
	std::vector<state, Eigen::aligned_allocator<state>> stack_vertex(frame_num_);
	std::stack<int> DFS;
	vertex_visit[0] = true;
	DFS.push(0);
	Eigen::Matrix4d ft = Eigen::Matrix4d::Identity();
	stack_vertex[0] = state(true, 0, 0, ft);
	while (!DFS.empty())
	{
		int item = DFS.top();
		bool add_stack = false;
		for (int i = stack_vertex[item].start_vertex; i < frame_num_; ++i)
		{
			if (graph[item][i].val == 1 && stack_vertex[i].in_stack == false)
			{
				DFS.push(i);

				int inx = graph[item][i].id;

				int last_level = stack_vertex[item].level;
				Eigen::Matrix4d t = stack_vertex[item].transformation_*traj_.data_[inx].transformation_;
				stack_vertex[i] = state(true, last_level + 1, 0, t);
				add_stack = true;

				stack_vertex[item].start_vertex = i + 1;

				vertex_visit[i] = true;

				init_seq[graph[item][i].id] = 1;
				break;
			}
		}
		if (!add_stack)
		{
			DFS.pop();
		}
	}

	//helpSeq_.Save("C:/Users/range/Desktop/local2global_init_seq.txt", init_seq, traj_);
	//ComputePose(init_seq);
	for (int i = 0; i < stack_vertex.size();++i)
		pose_.data_[i].transformation_ = stack_vertex[i].transformation_;
}

void ProgressiveOpt::ComputePose(const std::vector<int>& seq)
{
	g2o::SparseOptimizer* optimizer = new g2o::SparseOptimizer();
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
		v->setEstimate(Eigen2G2O(pose_.data_[i].transformation_));
		if (i == 0) {
			v->setFixed(true);
		}
		optimizer->addVertex(v);
	}
	RGBDInformation selected_edge;
	for (int i = 0; i < seq.size();++i)
	{
		if (seq[i]==1)
		{
			FramedTransformation & t = traj_.data_[i];
			FramedInformation & inf = info_.data_[i];
			selected_edge.data_.push_back(inf);

			g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3();
			g2o_edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex(t.frame1_));
			g2o_edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex(t.frame2_));
			g2o_edge->setMeasurement(g2o::internal::fromSE3Quat(Eigen2G2O(t.transformation_)));
			if (info_.data_.size() > 0) {
				g2o_edge->setInformation(info_.data_[i].information_);
			}
			else {
				g2o_edge->setInformation(default_information);
			}
			optimizer->addEdge(g2o_edge);
		}
	}
	optimizer->initializeOptimization();
	optimizer->optimize(100);
	for (int i = 0; i < pose_.data_.size(); i++) {
		g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3 *>(optimizer->vertex(i));
		pose_.data_[i].transformation_ = G2O2Matrix4d(v->estimateAsSE3Quat());
	}

	selected_edge.SaveToFile(selected_edge_file_);
}


void ProgressiveOpt::UnionFindSet(std::vector<int>& seq)
{
	int vertex_num = frame_num_;
	struct edge{
		int id_;
		int frame1_;
		int frame2_;
		Eigen::Matrix4d transformation_;

		double score;
		bool selected;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		edge(int frame1, int frame2, const Eigen::Matrix4d& t, double s)
			: frame1_(frame1), frame2_(frame2), transformation_(t), score(s), selected(false)
		{}

		edge() :id_(-1), frame1_(-1), frame2_(-1), transformation_(Eigen::Matrix4d::Identity()), score(-1.0), selected(false){}

		edge(const edge& e){
			id_ = e.id_;
			frame1_ = e.frame1_;
			frame2_ = e.frame2_;
			transformation_ = e.transformation_;
			score = e.score;
			selected = e.selected;
		}
	};

	struct cmp{
		bool operator()(edge& e1, edge& e2){
			return e1.score > e2.score;
		}
	};

	std::vector<edge, Eigen::aligned_allocator<edge>> graph;
	graph.resize(vertex_num*vertex_num);

	for (int i = 0; i < traj_.data_.size(); ++i)
	{
		int id1 = traj_.data_[i].frame1_;
		int id2 = traj_.data_[i].frame2_;

		edge e(id1, id2, traj_.data_[i].transformation_, info_.data_[i].score_);
		e.id_ = i;
		if (info_.data_[i].flag == 0)
			e.score *= 0.7;

		if (graph[id1*vertex_num + id2].id_ == -1)
		{
			graph[id1*vertex_num + id2] = e;
		}
		else
		{
			if (e.score>graph[id1*vertex_num + id2].score)
				graph[id1*vertex_num + id2] = e;
		}
	}


	// init select
	struct set_vertex
	{
		int set_id;
		std::vector<int> vertex_id;
	};
	std::vector<set_vertex> gather;			//from set find vertex
	gather.resize(vertex_num);
	for (int i = 0; i < vertex_num; ++i)
	{
		gather[i].set_id = i;
		gather[i].vertex_id.push_back(i);
	}
	std::vector<int> vertex2set;			//from vertex find set
	vertex2set.resize(vertex_num);
	for (int i = 0; i < vertex2set.size(); ++i)
		vertex2set[i] = i;

	for (int i = 0; i < seq.size();++i)
	{
		if (seq[i]==1)
		{
			int v1 = traj_.data_[i].frame1_;
			int v2 = traj_.data_[i].frame2_;
			//union
			if (vertex2set[v1]<vertex2set[v2])			// v2's set => v1's set
			{
				for (int t = 0; t < gather[vertex2set[v2]].vertex_id.size(); ++t)
				{
					gather[vertex2set[v1]].vertex_id.push_back(gather[vertex2set[v2]].vertex_id[t]);
				}
				for (int t = 0; t < gather[vertex2set[v2]].vertex_id.size(); ++t)
				{
					vertex2set[gather[vertex2set[v2]].vertex_id[t]] = vertex2set[v1];
				}
			}
			if (vertex2set[v1]>vertex2set[v2])
			{											// v1's set => v2's set
				for (int t = 0; t < gather[vertex2set[v1]].vertex_id.size(); ++t)
				{
					gather[vertex2set[v2]].vertex_id.push_back(gather[vertex2set[v1]].vertex_id[t]);
				}
				for (int t = 0; t < gather[vertex2set[v1]].vertex_id.size(); ++t)
				{
					vertex2set[gather[vertex2set[v1]].vertex_id[t]] = vertex2set[v2];
				}
			}
			graph[v1*vertex_num + v2].selected = true;
		}
	}
	for (int i = 0; i < vertex2set.size(); ++i)
	{
#ifdef Verbose
		std::cout << "vertex " << i << " belong set" << vertex2set[i] << "\n";
#endif
	}
	
	// select sequence edge
#ifdef Verbose
	std::cout << "The first select consecutive image (score_threshold: >" << score_threshold1_ << ")\n";
#endif
	for (int i = 1; i < vertex2set.size();++i)
	{
		if (vertex2set[i-1] !=vertex2set[i])
		{
			int v1 = i - 1;
			int v2 = i;
			if (graph[v1*vertex_num + v2].score>score_threshold1_)
			{
#ifdef Verbose
				std::cout << "\nChoose" << v1 << "----" << v2 << " to link all vertex\n";
#endif
				int id = graph[v1*vertex_num + v2].id_;
				seq[id] = 1;
				//union
				if (vertex2set[v1] < vertex2set[v2])		// v2's set => v1's set
				{
					for (int i = 0; i < gather[vertex2set[v2]].vertex_id.size(); ++i)
					{
						gather[vertex2set[v1]].vertex_id.push_back(gather[vertex2set[v2]].vertex_id[i]);
					}
					for (int i = 0; i < gather[vertex2set[v2]].vertex_id.size(); ++i)
					{
						vertex2set[gather[vertex2set[v2]].vertex_id[i]] = vertex2set[v1];
					}
				}
				if (vertex2set[v1] > vertex2set[v2])
				{											// v1's set => v2's set
					for (int i = 0; i < gather[vertex2set[v1]].vertex_id.size(); ++i)
					{
						gather[vertex2set[v2]].vertex_id.push_back(gather[vertex2set[v1]].vertex_id[i]);
					}
					for (int i = 0; i < gather[vertex2set[v1]].vertex_id.size(); ++i)
					{
						vertex2set[gather[vertex2set[v1]].vertex_id[i]] = vertex2set[v2];
					}
				}
				graph[v1*vertex_num + v2].selected = true;
			}
		}
	}
	std::sort(graph.begin(), graph.end(), cmp());
#ifdef Verbose
	std::cout << "\n\n\nThe second select higher score matching (score_threshold: >" << score_threshold2_ << ")\n";
#endif
	for (int index = 0; index < graph.size();++index)
	{
		// judge the same set or not
		int v1 = graph[index].frame1_;
		int v2 = graph[index].frame2_;
		if (v1 == -1 || v2 == -1)
			continue;
		if (vertex2set[v1] == vertex2set[v2])
			continue;
		if (graph[index].score < score_threshold2_)
			continue;
		
#ifdef Verbose
		std::cout << "\nChoose" << v1 << "----" << v2 << " to link all vertex\n";
#endif
		int id = graph[index].id_;
		seq[id] = 1;

		//union
		if (vertex2set[v1] < vertex2set[v2])		// v2's set => v1's set
		{
			for (int i = 0; i < gather[vertex2set[v2]].vertex_id.size(); ++i)
			{
				gather[vertex2set[v1]].vertex_id.push_back(gather[vertex2set[v2]].vertex_id[i]);
			}
			for (int i = 0; i < gather[vertex2set[v2]].vertex_id.size(); ++i)
			{
				vertex2set[gather[vertex2set[v2]].vertex_id[i]] = vertex2set[v1];
			}
		}
		if (vertex2set[v1] > vertex2set[v2])
		{											// v1's set => v2's set
			for (int i = 0; i < gather[vertex2set[v1]].vertex_id.size(); ++i)
			{
				gather[vertex2set[v2]].vertex_id.push_back(gather[vertex2set[v1]].vertex_id[i]);
			}
			for (int i = 0; i < gather[vertex2set[v1]].vertex_id.size(); ++i)
			{
				vertex2set[gather[vertex2set[v1]].vertex_id[i]] = vertex2set[v2];
			}
		}
		graph[index].selected = true;
	}

	// cannot connect all vertex
	if (!judgeLinkAllVertex(seq, traj_))
	{
		std::ofstream output(fail_file_);
		for (int i = 0; i < vertex2set.size(); ++i)
			output << "vertex " << i << " belong set" << vertex2set[i] << "\n";

		output.close();
#ifdef Verbose
		std::cout << "Graph cannot connect, see file " << fail_file_ << " \n";
#endif
		exit(-1);
	}
	
}


void ProgressiveOpt::UnionFindSetNonSequential(std::vector<int>& seq)
{
	int vertex_num = frame_num_;
	struct edge{
		int id_;
		int frame1_;
		int frame2_;
		Eigen::Matrix4d transformation_;

		double score;
		bool selected;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		edge(int frame1, int frame2, const Eigen::Matrix4d& t, double s)
			: frame1_(frame1), frame2_(frame2), transformation_(t), score(s), selected(false)
		{}

		edge() :id_(-1), frame1_(-1), frame2_(-1), transformation_(Eigen::Matrix4d::Identity()), score(-1.0), selected(false){}

		edge(const edge& e){
			id_ = e.id_;
			frame1_ = e.frame1_;
			frame2_ = e.frame2_;
			transformation_ = e.transformation_;
			score = e.score;
			selected = e.selected;
		}
	};

	struct cmp{
		bool operator()(edge& e1, edge& e2){
			return e1.score > e2.score;
		}
	};

	std::vector<edge, Eigen::aligned_allocator<edge>> graph;
	graph.resize(vertex_num*vertex_num);

	for (int i = 0; i < traj_.data_.size(); ++i)
	{
		int id1 = traj_.data_[i].frame1_;
		int id2 = traj_.data_[i].frame2_;

		edge e(id1, id2, traj_.data_[i].transformation_, info_.data_[i].score_);
		e.id_ = i;
		if (info_.data_[i].flag == 0)
			e.score *= 0.7;

		if (graph[id1*vertex_num + id2].id_ == -1)
		{
			graph[id1*vertex_num + id2] = e;
		}
		else
		{
			if (e.score>graph[id1*vertex_num + id2].score)
				graph[id1*vertex_num + id2] = e;
		}
	}


	// init select
	struct set_vertex
	{
		int set_id;
		std::vector<int> vertex_id;
	};
	std::vector<set_vertex> gather;			//from set find vertex
	gather.resize(vertex_num);
	for (int i = 0; i < vertex_num; ++i)
	{
		gather[i].set_id = i;
		gather[i].vertex_id.push_back(i);
	}
	std::vector<int> vertex2set;			//from vertex find set
	vertex2set.resize(vertex_num);
	for (int i = 0; i < vertex2set.size(); ++i)
		vertex2set[i] = i;

	for (int i = 0; i < seq.size(); ++i)
	{
		if (seq[i] == 1)
		{
			int v1 = traj_.data_[i].frame1_;
			int v2 = traj_.data_[i].frame2_;
			//union
			if (vertex2set[v1] < vertex2set[v2])			// v2's set => v1's set
			{
				for (int t = 0; t < gather[vertex2set[v2]].vertex_id.size(); ++t)
				{
					gather[vertex2set[v1]].vertex_id.push_back(gather[vertex2set[v2]].vertex_id[t]);
				}
				for (int t = 0; t < gather[vertex2set[v2]].vertex_id.size(); ++t)
				{
					vertex2set[gather[vertex2set[v2]].vertex_id[t]] = vertex2set[v1];
				}
			}
			if (vertex2set[v1]>vertex2set[v2])
			{											// v1's set => v2's set
				for (int t = 0; t < gather[vertex2set[v1]].vertex_id.size(); ++t)
				{
					gather[vertex2set[v2]].vertex_id.push_back(gather[vertex2set[v1]].vertex_id[t]);
				}
				for (int t = 0; t < gather[vertex2set[v1]].vertex_id.size(); ++t)
				{
					vertex2set[gather[vertex2set[v1]].vertex_id[t]] = vertex2set[v2];
				}
			}
			graph[v1*vertex_num + v2].selected = true;
		}
	}
	for (int i = 0; i < vertex2set.size(); ++i)
	{
#ifdef Verbose
		std::cout << "vertex " << i << " belong set" << vertex2set[i] << "\n";
#endif
	}

	std::sort(graph.begin(), graph.end(), cmp());
#ifdef Verbose
	std::cout << "\n\n\nThe second select higher score matching (score_threshold: >" << score_threshold2_ << ")\n";
#endif
	for (int index = 0; index < graph.size(); ++index)
	{
		// judge the same set or not
		int v1 = graph[index].frame1_;
		int v2 = graph[index].frame2_;
		if (v1 == -1 || v2 == -1)
			continue;
		if (vertex2set[v1] == vertex2set[v2])
			continue;
		if (graph[index].score < score_threshold2_)
			continue;

#ifdef Verbose
		std::cout << "\nChoose" << v1 << "----" << v2 << " to link all vertex\n";
#endif
		int id = graph[index].id_;
		seq[id] = 1;

		//union
		if (vertex2set[v1] < vertex2set[v2])		// v2's set => v1's set
		{
			for (int i = 0; i < gather[vertex2set[v2]].vertex_id.size(); ++i)
			{
				gather[vertex2set[v1]].vertex_id.push_back(gather[vertex2set[v2]].vertex_id[i]);
			}
			for (int i = 0; i < gather[vertex2set[v2]].vertex_id.size(); ++i)
			{
				vertex2set[gather[vertex2set[v2]].vertex_id[i]] = vertex2set[v1];
			}
		}
		if (vertex2set[v1] > vertex2set[v2])
		{											// v1's set => v2's set
			for (int i = 0; i < gather[vertex2set[v1]].vertex_id.size(); ++i)
			{
				gather[vertex2set[v2]].vertex_id.push_back(gather[vertex2set[v1]].vertex_id[i]);
			}
			for (int i = 0; i < gather[vertex2set[v1]].vertex_id.size(); ++i)
			{
				vertex2set[gather[vertex2set[v1]].vertex_id[i]] = vertex2set[v2];
			}
		}
		graph[index].selected = true;
	}

	// cannot connect all vertex
	if (!judgeLinkAllVertex(seq, traj_))
	{
		std::ofstream output(fail_file_);
		for (int i = 0; i < vertex2set.size(); ++i)
			output << "vertex " << i << " belong set" << vertex2set[i] << "\n";

		output.close();

		std::cout << "Graph cannot connect, see file " << fail_file_ << " \n";
		exit(-1);
	}

}