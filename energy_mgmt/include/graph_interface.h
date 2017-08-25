#ifndef GRAPH_INTERFACE_H
#define GRAPH_INTERFACE_H

#include <vector>

class GraphInterface
{
public:
    virtual void addNode(unsigned int node) = 0;
    virtual void addEdge(unsigned int node_A, unsigned int node_B, double cost) = 0;
    virtual double getEdgeCost(unsigned int node_A, unsigned int node_B) = 0;
    virtual std::vector<unsigned int> getNodeVector() = 0;
    
protected:
    std::vector< std::vector<double> > graph;

};

#endif // GRAPH_INTERFACE_H
