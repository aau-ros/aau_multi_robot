#include <ros/ros.h>
#include <gtest/gtest.h>
#include "graph.h"

TEST(TestGraph, testAddNodeAndNodeExists)
{
    Graph graph;
    graph.addNode(5);
    EXPECT_TRUE(graph.nodeExists(5));
    EXPECT_FALSE(graph.nodeExists(2));
}

TEST(TestGraph, testGetNodeVector)
{
    Graph graph;
    graph.addNode(5);
    graph.addNode(7);
    graph.addNode(2);
    std::vector<unsigned int> nodes = graph.getNodeVector();
    ASSERT_EQ(3, nodes.size());
    EXPECT_EQ(2, nodes.at(0));
    EXPECT_EQ(5, nodes.at(1));
    EXPECT_EQ(7, nodes.at(2));
}

TEST(TestGraph, testGetEdgeCost)
{
    Graph graph;

    graph.addNode(5);
    graph.addNode(2);
    EXPECT_EQ( 0, graph.getEdgeCost(2, 2));
    EXPECT_EQ(-1, graph.getEdgeCost(2, 5));
    EXPECT_EQ(-1, graph.getEdgeCost(5, 2));

    graph.addNode(7);
    EXPECT_EQ( 0, graph.getEdgeCost(2, 2));
    EXPECT_EQ(-1, graph.getEdgeCost(2, 5));
    EXPECT_EQ(-1, graph.getEdgeCost(5, 2));
    EXPECT_EQ(-1, graph.getEdgeCost(2, 7));
    EXPECT_EQ(-1, graph.getEdgeCost(7, 2));
    EXPECT_EQ( 0, graph.getEdgeCost(5, 5));
    EXPECT_EQ(-1, graph.getEdgeCost(5, 7));
    EXPECT_EQ(-1, graph.getEdgeCost(7, 5));

    graph.addEdge(2, 7, 10);
    graph.addEdge(5, 7, 20);
    EXPECT_EQ( 0, graph.getEdgeCost(2, 2));
    EXPECT_EQ(-1, graph.getEdgeCost(2, 5));
    EXPECT_EQ(-1, graph.getEdgeCost(5, 2));
    EXPECT_EQ(10, graph.getEdgeCost(2, 7));
    EXPECT_EQ(10, graph.getEdgeCost(7, 2));
    EXPECT_EQ( 0, graph.getEdgeCost(5, 5));
    EXPECT_EQ(20, graph.getEdgeCost(5, 7));
    EXPECT_EQ(20, graph.getEdgeCost(7, 5));
    EXPECT_EQ( 0, graph.getEdgeCost(7, 7));

    graph.addNode(9);
    graph.addEdge(2, 9, 30);
    EXPECT_EQ( 0, graph.getEdgeCost(2, 2));
    EXPECT_EQ(-1, graph.getEdgeCost(2, 5));
    EXPECT_EQ(-1, graph.getEdgeCost(5, 2));
    EXPECT_EQ(10, graph.getEdgeCost(2, 7));
    EXPECT_EQ(10, graph.getEdgeCost(7, 2));
    EXPECT_EQ(30, graph.getEdgeCost(2, 9));
    EXPECT_EQ(30, graph.getEdgeCost(9, 2));
    EXPECT_EQ( 0, graph.getEdgeCost(5, 5));
    EXPECT_EQ(20, graph.getEdgeCost(5, 7));
    EXPECT_EQ(20, graph.getEdgeCost(7, 5));
    EXPECT_EQ(-1, graph.getEdgeCost(5, 9));
    EXPECT_EQ(-1, graph.getEdgeCost(9, 5));
    EXPECT_EQ( 0, graph.getEdgeCost(7, 7));
    EXPECT_EQ(-1, graph.getEdgeCost(7, 9));
    EXPECT_EQ(-1, graph.getEdgeCost(9, 7));
    EXPECT_EQ( 0, graph.getEdgeCost(9, 9));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "graph_utest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
