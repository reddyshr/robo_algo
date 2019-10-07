
#include <ros/ros.h>
#include "DirectedGraph.hpp"
#include <vector>

void init_graph(DirectedGraph& dg, std::vector<float>& x_coor, std::vector<float>& y_coor) {

	std::vector<float> node_coor {0.0, 0.0};

	for (int i = 0; i < dg.getSize(); i++) {

		node_coor[0] = x_coor[i];
		node_coor[1] = y_coor[i];

		dg.addNodeCoordinate(i, node_coor);
	}

}

int main(int argc, char** argv) {


	ros::init(argc, argv, "algo_executer");
	ros::NodeHandle nh;


	std::vector<float> x_coor;
	std::vector<float> y_coor;

	nh.getParam("/graph/x_coor", x_coor);
	nh.getParam("/graph/y_coor", y_coor);

	DirectedGraph dg {x_coor.size(), 2};
	init_graph(dg, x_coor, y_coor);

	while (ros::ok()) {

	}



}