#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <unordered_map>
#include <algorithm>    // std::sort

#include <ros/ros.h>

class Graph
{
public:
    Graph();
    void addNode(unsigned int node);
    bool nodeExists(unsigned int node);
    void addEdge(unsigned int node_A, unsigned int node_B, double cost);
    double getEdgeCost(unsigned int node_A, unsigned int node_B);
    std::vector<unsigned int> getNodeVector();

private:
    std::vector<unsigned int> nodes; //TODO use unordere_map
//    std::unordered_map<unsigned int, std::unordered_map<unsigned int, double> > graph; //TODO use this
    std::vector< std::vector<double> > graph;
};

#endif // GRAPH_H
