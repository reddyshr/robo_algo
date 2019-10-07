#ifndef ROBO_ALGO_GRAPH_HPP
#define ROBO_ALGO_GRAPH_HPP

#include <vector>
#include <list>
#include <unordered_map>
#include <map>
#include <cmath>

class Graph {

protected:

	std::vector<std::list<int>> m_graph;
	std::unordered_map<int, std::vector<float>> m_node_coordinate_map;
	int m_graph_size;
	int m_coordinate_dim;

	bool validNode(int node);

public:

	Graph(int num_of_nodes, int coordinate_dimension);

	int getSize();

	void addNodeCoordinate(int node, std::vector<float> coor);

};



#endif