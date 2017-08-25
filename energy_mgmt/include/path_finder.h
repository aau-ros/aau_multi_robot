#ifndef PATH_FINDER_H
#define PATH_FINDER_H

#include <vector>
#include <ros/ros.h>
#include "graph.h"

class PathFinder
{
public:
    PathFinder();
    void setGraph(Graph *graph);
    bool findShortestPath(unsigned int start, unsigned int end, std::vector<unsigned int> &path);
    
private:
    Graph *graph;

};

#endif // PATH_FINDER_H
