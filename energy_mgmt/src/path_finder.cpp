#include "path_finder.h"
#include <limits.h>

PathFinder::PathFinder() {}

// TODO(minor) does not work if a DS is not connected to any other DS
//TODO should be refactored
bool PathFinder::findShortestPath(unsigned int start, unsigned int end, std::vector<unsigned int> &path)
{
    path.clear();

    if(start == end) {
        ROS_WARN("Starting node coincides with ending node");
        path.push_back(start);
        return true;
    }

    std::vector<unsigned int> nodes = graph->getNodeVector(); //TODO how to avoid the copy AND to handle the case of graph modified during path computation?
    unsigned int V = nodes.size();

    // In these arrays, the index correspond to node nodes[index] (e.g.: index 1 -> node nodes[1] = 3);
    int parents[V];     // Array to store constructed MST
    float keys[V];      // Key values used to pick minimum weight edge in cut
    bool mstSet[V];     // To represent set of vertices not yet included in MST
    
    // Initialize all keys as INFINITE
    for (unsigned int i = 0; i < V; i++) {
        keys[i] = INT_MAX;
        mstSet[i] = false;
        parents[i] = -1;
    }

    for(unsigned int i=0; i < nodes.size(); i++)
        if(nodes[i] == start) {
            keys[i] = 0;    // Make key 0 so that this vertex is picked as first vertex
            break;
        }

    bool end_node_reached = false, path_found = false;
    unsigned int count;
    for (count = 0; count < V - 1 && !path_found; count++)
    {
        // Pick the minimum key vertex from the set of vertices
        // not yet included in MST
        int min = INT_MAX, u = -1;
        for (unsigned int v = 0; v < V; v++)
            if (mstSet[v] == false && keys[v] < min) {
                min = keys[v];
                u = v;
            }

        if(nodes[u] == end)
            path_found = true;
    
        else {
            // Add the picked vertex to the MST Set
            mstSet[u] = true;
            
            // Update key value and parent index of the adjacent vertices of
            // the picked vertex. Consider only those vertices which are not yet
            // included in MST
            for (unsigned int v = 0; v < V; v++) {
                // graph[u][v] is non zero only for adjacent vertices of m
                // mstSet[v] is false for vertices not yet included in MST
                // Update the key only if graph[u][v] is smaller than key[v]
                double cost = graph->getEdgeCost(nodes[u], nodes[v]);
                if (cost > 0 && !mstSet[v] && (keys[u] + cost < keys[v])) {
                    parents[v] = u;
                    keys[v] = keys[u] + cost;
                    if(nodes[v] == end)
                        end_node_reached = true;
                }
            }
        }
    }

    // Construct path from 'start' to 'end', if it exists
    if(end_node_reached) {
        unsigned int index;
        for(unsigned int i=0; i < nodes.size(); i++)
            if(nodes[i] == end) {
                index = i;
                break;
        }

        path.push_back(nodes[index]);
        while(nodes[index] != start) {
            index = parents[index];
            path.push_back(nodes[index]);
        }

        std::reverse(path.begin(), path.end());
    }

    return end_node_reached;
}

void PathFinder::setGraph(Graph *graph) {
    this->graph = graph;
}
