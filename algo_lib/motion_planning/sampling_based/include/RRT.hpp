#ifndef ROBO_ALGO_RRT_HPP
#define ROBO_ALGO_RRT_HPP

#include <vector>
#include <map>
#include <cmath>
#include <limits>

class RRT {

private:

	int m_num_of_nodes;
	int m_max_num_of_nodes;
	std::vector<int> m_parent_nodes;
	std::map<int, vector<float>> m_node_coordinate_map;

	int findNearestNode(vector<float>& node_coord);

	float computeDistance(std::vector<float>& node1, std::vector<float> node2);

	bool withinGoalRegion(std::vector<float> curr_node, std::vector<float> tgt_node, std::vector<float> goal_region_dim);

	std::vector<float> getSample(std::vector<float> lower_bound, std::vector<float> upper_bound);

	std::vector<float> findNewNode(std::vector<float>& nearest_node, std::vector<float>& sample_node, float d);

public:

	RRT();

	void reset();

	std::list<std::vector<float>> run(std::vector<float> src_node, std::vector<float> tgt_node, std::vector<float> lower_bound, std::vector<float> upper_bound, std::vector<float> goal_region_dim, int max_tree_size, float d);

}





#endif
