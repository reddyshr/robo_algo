#include "astar.hpp"


void runAstar(DirectedGraph& dg, int src_node, int tgt_node, std::list<int>& computed_path_out) {

	int numOfNodes = dg.getSize();

	std::set<std::vector<float>, DistanceCompare> open_set;
	std::set<int> closed_set;
	std::vector<int> parent_node {numOfNodes, -1};
	parent_node[src_node] = src_node;

	vector<float> curr_cost {numOfNodes, std::numeric_limits<float>::infinity()};
	curr_cost[src_node] = 0.0;

	open_set.insert(std::vector<float> {std::static_cast<float>(src_node), curr_cost[src_node]});

	while (!open_set.empty()) {

		std::vector<float> curr_node = (*open_set.begin());
		open_set.erase(curr_node);
		closed_set.insert{curr_node[0]};

		if (curr_node[0] == tgt_node) {
			break;
		}

		std::list<int> neighbors = getNeighbors(curr_node[0]);

		for (auto nbr : neighbors) {
			if (closed_set.find(nbr) != neighbors.end()) {

				float tentative_curr_cost = curr_cost[curr_node[0]] + dg.getEdgeWeight(curr_node[0], nbr);

				if (tentative_curr_cost < curr_cost[nbr]) {

					float heuristic_cost = computeHeuristicCost(dg, nbr, tgt_node);
					float curr_estimated_cost = curr_cost[nbr] + heuristic_cost; 

					std::vector<float> nbr_node {nbr, curr_estimated_cost};
					if (open_set.find(nbr_node) != open_set.end()) {
						open_set.erase(nbr_node);
					}

					nbr_node[1] = tentative_curr_cost + heuristic_cost;
					open_set.insert(nbr_node);

					curr_cost[nbr] = tentative_curr_cost;
					parent_node[nbr] = curr_node[0];

				}
			}
		}
	}

	if (parent_node[tgt_node] == -1) {
		//No Path Found From src_node to tgt_node
	} else {

		int n = tgt_node;

		while (n != src_node) {
			computed_path_out.push_front(n);
			n = parent_node[n];
		}

		computed_path_out.push_front(src_node);

	}

	return;



}

