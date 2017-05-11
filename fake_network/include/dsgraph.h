#ifndef DSGRAPH_H
#define DSGRAPH_H

#include <iostream>
#include <stdlib.h>

class DsGraph
{
    public:
        DsGraph() {
            num_nodes = 0;
            for(int i=0; i<100; i++)
                for(int j=0; j<100; j++)
                    graph[i][j] = -1;
        }
    
        void addEdge(int start, int end, float cost) {
            if(start >= num_nodes)
                num_nodes = start + 1;
            if(end >= num_nodes)
                num_nodes = end + 1;
            graph[start][end] = cost;
            graph[end][start] = cost;
        
        };
        
        bool hasEdge(int start, int end);
        bool findPath(int start, int end, std::vector<int> &path);
        
        void print() {

            for(int i=0; i<num_nodes; i++) {
                for(int j=0; j<num_nodes; j++)
                    ROS_ERROR("%.2f", graph[i][j]);
                }
        };
    
    private:
        int num_nodes;
        float graph[100][100];
};

#endif /* DSGRAPH_H */
