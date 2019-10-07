#include "DirectedGraph.hpp"

//PUBLIC FUNCTIONS

DirectedGraph::DirectedGraph(int numOfNodes = 0, int coordinate_dimension = 2) : Graph(numOfNodes, coordinate_dimension) {

}

bool DirectedGraph::directedEdgeExists(int src_node, int tgt_node) {

	if (!validNode(src_node) || !validNode(tgt_node)) {
		return false;
	}

	for (auto n : m_graph[src_node]) {
		if (n == tgt_node) {
			return true;
		}
	}

	return false;

}

bool DirectedGraph::addDirectedEdge(int src_node, int tgt_node) {

	if (!validNode(src_node) || !validNode(tgt_node) || directedEdgeExists(src_node, tgt_node)) {

		return false;

	} else {

		m_graph[src_node].push_front(tgt_node);
		float weight = DirectedGraph::computeDistance(src_node, tgt_node);
		m_edge_weight_map.insert({std::vector<int> {src_node, tgt_node}, weight});

	}

}

bool DirectedGraph::addDirectedEdge(int src_node, int tgt_node, float weight) {

	if (!validNode(src_node) || !validNode(tgt_node) || directedEdgeExists(src_node, tgt_node)) {

		return false;

	} else {

		m_graph[src_node].push_front(tgt_node);
		m_edge_weight_map.insert({std::vector<int> {src_node, tgt_node}, weight});
	}

}

bool DirectedGraph::removeDirectedEdge(int src_node, int tgt_node) {

	if (!directedEdgeExists(src_node, tgt_node)) {
		return false;
	} else {
		m_graph[src_node].remove(tgt_node);
		m_edge_weight_map.erase(std::vector<int> {src_node, tgt_node});
	}

}

std::list<int> DirectedGraph::getNeighbors(int node) {

	if (validNode(node)) {
		return m_graph[node];
	} else {
		std::list<int> empty_list {};
		return empty_list;
	}

}


float DirectedGraph::computeDistance(int src_node, int tgt_node) {

	if (!validNode(src_node) || !validNode(tgt_node)) {
		return -1.0;
	}

	std::vector<float> src_coord = m_node_coordinate_map[src_node];
	std::vector<float> tgt_coord = m_node_coordinate_map[tgt_node];

	float dist = 0;

	for (int i = 0; i < src_coord.size(); i++) {

		dist = dist + std::pow(std::abs(src_coord[i] - tgt_coord[i]), 2);

	}

	return std::sqrt(dist);

}


float getEdgeWeight(int src_node, int tgt_node) {

	return *m_edge_weight_map.find(std::vector<int> {src_node, tgt_node});

}
