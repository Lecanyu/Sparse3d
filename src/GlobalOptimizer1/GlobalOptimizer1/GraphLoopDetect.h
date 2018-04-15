#include <stack>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cassert>

#include "helper.h"


class GraphLoopDetect
{
	int frame_num_;
public:
	std::vector<std::vector<int>> graph;

	// result variable, it will be filled after run DetectLoop()
	std::vector<std::vector<int>> edge_loop_num;
	std::vector<std::vector<int>> final_result;


	GraphLoopDetect(){}
	~GraphLoopDetect(){}

	bool Init(std::string& traj_file, int frame_num)
	{
		frame_num_ = frame_num;

		RGBDTrajectory traj;
		traj.LoadFromFile(traj_file);

		graph.resize(frame_num, std::vector<int>(frame_num, 0));

		for (int i = 0; i < traj.data_.size(); ++i)
		{
			int v1 = traj.data_[i].frame1_;
			int v2 = traj.data_[i].frame2_;

			graph[v1][v2] = 1;
			graph[v2][v1] = 1;
		}

		return traj.data_.size() > 0;
	}


	// 
	void  DetectLoop()
	{
		std::cout << "Detecting...";
		struct state{
			bool in_stack;
			int level;
			int start_vertex;
			state() :in_stack(false), level(-1), start_vertex(0){}
			state(bool ins, int le, int start) :in_stack(ins), level(le), start_vertex(start){}
		};
		std::vector<state> stack_vertex(frame_num_);

		// save coarse result
		std::vector<std::vector<int>> loop_result;

		// DFS
		std::stack<int> DFS;
		DFS.push(0);
		stack_vertex[0] = state(true, 0, 0);
		while (!DFS.empty())
		{
			int item = DFS.top();
			bool add_stack = false;
			for (int i = stack_vertex[item].start_vertex; i < frame_num_; ++i)
			{
				if (graph[item][i] == 1 && stack_vertex[i].in_stack == false)
				{
					DFS.push(i);

					int last_level = stack_vertex[item].level;
					stack_vertex[i] = state(true, last_level + 1, 0);
					add_stack = true;

					stack_vertex[item].start_vertex = i + 1;
					break;
				}
				if (graph[item][i] == 1 && stack_vertex[i].in_stack == true)
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

						loop_result.push_back(loop);
						//std::cout << "Coarse loop number: " << loop_result.size() << "\n";
					}
				}
			}
			if (!add_stack)
			{
				DFS.pop();
				stack_vertex[item].in_stack = false;
				stack_vertex[item].level = -1;
				stack_vertex[item].start_vertex = 0;
			}
		}

		for (int i = 0; i < loop_result.size(); ++i)
		{
			bool repeat = false;
			for (int t = 0; t < final_result.size(); ++t)
			{
				if (loop_eq(loop_result[i], final_result[t]))
				{
					repeat = true;
					break;
				}
			}

			if (!repeat)
				final_result.push_back(loop_result[i]);
		}

		for (int i = 0; i < final_result.size(); ++i)
			final_result[i].push_back(final_result[i][0]);

		// count 
		edge_loop_num.resize(frame_num_, std::vector<int>(frame_num_, 0));
		for (int i = 0; i < final_result.size(); ++i)
		{
			for (int j = 1; j < final_result[i].size(); ++j)
			{
				int v1 = final_result[i][j - 1];
				int v2 = final_result[i][j];

				if (v1 > v2)
				{
					int temp = v1;
					v1 = v2;
					v2 = temp;
				}
				edge_loop_num[v1][v2]++;
			}
		}
		std::cout << " Done!\n";
	}


private:
	// utility function
	// Judge array loop equation (clockwise and counterclockwise)
	bool loop_eq(std::vector<int>& v1, std::vector<int>& v2)
	{
		if (v1.size() != v2.size())
			return false;

		std::vector<int> temp = v1;
		temp.insert(temp.end(), v1.begin(), v1.end());

		// clockwise
		int i = 0;
		for (; i < temp.size(); ++i)
		{
			if (temp[i] != v2[0])
				continue;
			else
				break;
		}

		int count = 0;
		for (int j = 0; j < v2.size() && i < temp.size(); ++i, ++j)
			if (temp[i] == v2[j])
				count++;

		if (count == v2.size())
			return true;
		else
		{	// counterclockwise
			i = temp.size() - 1;
			for (; i >= 0; --i)
				if (temp[i] != v2[0])
					continue;
				else
					break;
			count = 0;
			for (int j = 0; j < v2.size() && i >= 0; --i, ++j)
				if (temp[i] == v2[j])
					count++;
			if (count == v2.size())
				return true;
			else
				return false;
		}
	}
};