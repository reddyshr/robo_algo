#include "Graph.hpp"

Graph::Graph(int num_of_nodes = 0, int coordinate_dimension = 2) : m_graph(num_of_nodes), m_graph_size{num_of_nodes}, m_coordinate_dim{coordinate_dimension},  m_node_coordinate_map{} {

}

bool Graph::validNode(int node) {

	return (node >=0 && node < m_graph_size);

}

int Graph::getSize() {
	return m_graph_size;
}

void Graph::addNodeCoordinate(int node, std::vector<float> coor) {
	m_node_coordinate_map.insert({node, coor});
}
