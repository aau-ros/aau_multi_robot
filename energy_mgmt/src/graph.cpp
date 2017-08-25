#include "graph.h"

Graph::Graph() {}

void Graph::addNode(unsigned int node) {
    while(graph.size() <= node) {
        std::vector<double> tmp;
        graph.push_back(tmp);
    }

    for(auto it = graph.begin(); it != graph.end(); it++)
        while(it->size() <= node)
            it->push_back(-1);

    graph.at(node).at(node) = 0;

    nodes.push_back(node);
    std::sort(nodes.begin(), nodes.end());
}

void Graph::addEdge(unsigned int node_A, unsigned int node_B, double cost) {
    if(node_A == node_B) {
        ROS_ERROR("node_A and node_B are equal, this is not allowed: ignoring call");
        return;
    }

    if(!nodeExists(node_A))
        addNode(node_A);
    if(!nodeExists(node_B))
        addNode(node_B);

    graph.at(node_A).at(node_B) = cost;
    graph.at(node_B).at(node_A) = cost;
}

bool Graph::nodeExists(unsigned int node) {
    for(unsigned int i = 0; i < nodes.size(); i++)
        if(nodes.at(i) == node)
            return true;
    return false;
}

double Graph::getEdgeCost(unsigned int node_A, unsigned int node_B) {
    if(!nodeExists(node_A) || !nodeExists(node_B))
        return -1;
    return graph.at(node_A).at(node_B);
}

std::vector<unsigned int> Graph::getNodeVector() {
    return nodes;
}
