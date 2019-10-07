#include "RRT.hpp"

RRT::RRT() {
	m_num_of_nodes = 0;
}


void RRT::reset() {

	m_num_of_nodes = 0;
	m_parent_nodes.clear();
	m_node_coordinate_map.clear();

}

std::list<std::vector<float>> RRT::run(std::vector<float> src_node, std::vector<float> tgt_node, std::vector<float> lower_bound, std::vector<float> upper_bound, std::vector<float> goal_region_dim, int max_tree_size, float d) {

	m_node_coordinate_map.insert(0, src_node);

	std::vector<float> sample_coor;
	std::vector<float> new_node_coor;
	int nearest_node;
	bool path_found;

	while ((m_num_of_nodes < max_tree_size) && !path_found) {

		m_num_of_nodes++;

		sample_coor = RRT::getSample(lower_bound, upper_bound);

		nearest_node = RRT::findNearestNode(sample_coor);

		new_node_coor = findNewNode((*m_node_coordinate_map.find(nearest_node)), sample_coor, d);

		m_node_coordinate_map.insert(m_num_of_nodes, new_node_coor);

		m_parent_nodes[m_num_of_nodes] = nearest_node;

		if (withinGoalRegion(new_node_coor, tgt_node, goal_region_dim)) {
			path_found = true;
		}
	}

	std::list<std::vector<float>> path {};

	if (path_found) {

		int curr_node = m_num_of_nodes;

		while (curr_node != 0) {

			path.push_front(*m_node_coordinate_map.find(curr_node));

			curr_node = m_parent_nodes[curr_node];

		}

		path.push_front(src_node);

	}

	return path;

}

int RRT::findNearestNode(vector<float>& node_coord) {

	float min_dist = std::numeric_limits<float>::max();
	int nearest_node = -1;

	std::map<int, vector<float>>::iterator it;

	for (it = m_node_coordinate_map.begin(); it != m_node_coordinate_map.end(); it++) {

		float curr_dist = RRT::computeDistance(node_coord, it->second);

		if (curr_dist < min_dist) {
			min_dist = curr_dist;
			nearest_node = it->first;
		}

	}

	return nearest_node;


}

float RRT::computeDistance(std::vector<float>& src_coord, std::vector<float> src_coord) {

	float dist = 0;

	for (int i = 0; i < src_coord.size(); i++) {

		dist = dist + std::pow(std::abs(src_coord[i] - tgt_coord[i]), 2);

	}

	return std::sqrt(dist);

}

bool RRT::withinGoalRegion(std::vector<float> curr_node, std::vector<float> tgt_node, std::vector<float> goal_region_dim) {

	for (int i = 0; i < curr_node.size(); i++) {

		if (std::abs(curr_node[i] - tgt_node[i]) >= goal_region_dim[i]) {
			return false;
		}

	}

	return true;

}


std::vector<float> RRT::getSample(std::vector<float> lower_bound, std::vector<float> upper_bound) {

	std::vector<float> sample {lower_bound.size(), 0.0};

	for (int i = 0; i < sample.size(); i++) {

		float random_num = ((float) std::rand()) / (float) std::RAND_MAX;
		float range = upper_bound[i] - lower_bound[i];
		sample[i] = (lower_bound[i] + range * random_num);

	}

	return sample;

}


std::vector<float> RRT::findNewNode(std::vector<float>& nearest_node, std::vector<float>& sample_node, float d) {

	std::vector<float> unit_slope {};
	std::vector<float> new_node {};
	float length = computeDistance(nearest_node, sample_node);

	for (int i = 0; i < nearest_node.size(); i++) {

		unit_slope[i] = (sample_node[i] - nearest_node[i]) / length;
		new_node[i] = d * unit_slope[i] + nearest_node[i];

	}

	return new_node;

}