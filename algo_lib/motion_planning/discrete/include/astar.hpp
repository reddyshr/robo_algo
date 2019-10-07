#ifndef ROBO_ALGO_ASTAR_HPP
#define ROBO_ALGO_ASTAR_HPP

#include <list>
#include <set>
#include <cmath>

#include "DirectedGraph.hpp"

enum ASTAR_HEURISTIC{NONE, STRAIGHTLINE};


struct DistanceCompare {

	bool operator()(const vector<float>& first_node, const vector<float>& second_node) {
		return first_node[1] < second_node[1];
	}

};

float computeHeuristicCost(DirectedGraph& dg, int src_node, int tgt_node);

void runAstar(DirectedGraph& dg, int src_node, int tgt_node, std::list<int>& computed_path_out);



#endif