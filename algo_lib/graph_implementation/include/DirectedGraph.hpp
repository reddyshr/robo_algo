#ifndef ROBO_ALGO_DIRECTED_GRAPH_HPP
#define ROBO_ALGO_DIRECTED_GRAPH_HPP

#include "Graph.hpp"

class DirectedGraph : public Graph {

private:

	std::map<std::vector<int>, float> m_edge_weight_map;

public:

	DirectedGraph(int numOfNodes, int coordinate_dimension);

	bool directedEdgeExists(int src_node, int tgt_node);

	bool addDirectedEdge(int src_node, int tgt_node);

	bool addDirectedEdge(int src_node, int tgt_node, float weight);

	bool removeDirectedEdge(int src_node, int tgt_node);

	std::list<int> getNeighbors(int node);

	float computeDistance(int src_node, int tgt_node);

	float getEdgeWeight(int src_node, int tgt_node);


};


#endif