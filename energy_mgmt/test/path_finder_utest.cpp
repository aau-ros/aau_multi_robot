#include <gtest/gtest.h>
#include "path_finder.h"
#include "graph.h"

// Check other/graph.jpg for a picture of the used graph
TEST(TestPathFinder, testFindShortestPath)
{
    PathFinder pf;
    Graph graph;
    pf.setGraph(&graph);

    graph.addEdge(0, 1, 3);
    graph.addEdge(0, 7, 8);
    graph.addEdge(1, 2, 8);
    graph.addEdge(1, 7, 11);
    graph.addEdge(2, 3, 7);
    graph.addEdge(2, 5, 4);
    graph.addEdge(2, 8, 2);
    graph.addEdge(3, 4, 9);
    graph.addEdge(3, 5, 13);
    graph.addEdge(4, 5, 10);
    graph.addEdge(4, 9, 1);
    graph.addEdge(5, 6, 2);
    graph.addEdge(6, 7, 1);
    graph.addEdge(7, 8, 7);

    std::vector<unsigned int> path;

    ASSERT_TRUE(pf.findShortestPath(0, 4, path));
    ASSERT_EQ(5, path.size());
    EXPECT_EQ(0, path.at(0));
    EXPECT_EQ(7, path.at(1));
    EXPECT_EQ(6, path.at(2));
    EXPECT_EQ(5, path.at(3));
    EXPECT_EQ(4, path.at(4));

    ASSERT_TRUE(pf.findShortestPath(4, 0, path));
    ASSERT_EQ(5, path.size());
    EXPECT_EQ(4, path.at(0));
    EXPECT_EQ(5, path.at(1));
    EXPECT_EQ(6, path.at(2));
    EXPECT_EQ(7, path.at(3));
    EXPECT_EQ(0, path.at(4));

    ASSERT_TRUE(pf.findShortestPath(0, 8, path));
    ASSERT_EQ(4, path.size());
    EXPECT_EQ(0, path.at(0));
    EXPECT_EQ(1, path.at(1));
    EXPECT_EQ(2, path.at(2));
    EXPECT_EQ(8, path.at(3));
}

TEST(TestPathFinder, testFindShortestPath2)
{
    PathFinder pf;
    Graph graph;
    pf.setGraph(&graph);

    graph.addEdge(0, 1, 3);
    graph.addEdge(0, 7, 8);
//    graph.addEdge(1, 2, 8);
//    graph.addEdge(1, 7, 11);
    graph.addEdge(2, 3, 7);
    graph.addEdge(2, 5, 4);
    graph.addEdge(2, 8, 2);
    graph.addEdge(3, 4, 9);
    graph.addEdge(3, 5, 13);
    graph.addEdge(4, 5, 10);
    graph.addEdge(4, 9, 1);
    graph.addEdge(5, 6, 2);
    graph.addEdge(6, 7, 1);
    graph.addEdge(7, 8, 7);

    std::vector<unsigned int> path;
    ASSERT_TRUE(pf.findShortestPath(0, 4, path));
    ASSERT_EQ(5, path.size());
    EXPECT_EQ(0, path.at(0));
    EXPECT_EQ(7, path.at(1));
    EXPECT_EQ(6, path.at(2));
    EXPECT_EQ(5, path.at(3));
    EXPECT_EQ(4, path.at(4));
}

TEST(TestPathFinder, testFindShortestPathWithNonSequentialIdentifiers)
{
    PathFinder pf;
    Graph graph;
    pf.setGraph(&graph);

    graph.addEdge(1, 3, 4);
    graph.addEdge(1, 7, 10);
    graph.addEdge(3, 7, 5);

    std::vector<unsigned int> path;
    ASSERT_TRUE(pf.findShortestPath(1, 7, path));
    ASSERT_EQ(3, path.size());
    EXPECT_EQ(1, path.at(0));
    EXPECT_EQ(3, path.at(1));
    EXPECT_EQ(7, path.at(2));
}

TEST(TestPathFinder, testFindShortestPathWithUnconnectedComponents)
{
    PathFinder pf;
    Graph graph;
    pf.setGraph(&graph);

    graph.addEdge(0, 1, 3);
    graph.addEdge(0, 7, 8);
    graph.addEdge(1, 2, 8);
    graph.addEdge(1, 7, 11);
    graph.addEdge(2, 3, 7);
    graph.addEdge(2, 5, 4);
    graph.addEdge(2, 8, 2);
//    graph.addEdge(3, 4, 9);
//    graph.addEdge(3, 5, 13);
//    graph.addEdge(4, 5, 10);
//    graph.addEdge(4, 9, 1);
    graph.addEdge(5, 6, 2);
    graph.addEdge(6, 7, 1);
    graph.addEdge(7, 8, 7);

    std::vector<unsigned int> path;

    ASSERT_FALSE(pf.findShortestPath(0, 4, path));
}

TEST(TestPathFinder, testFindShortestPathWithUnconnectedComponents2)
{
    PathFinder pf;
    Graph graph;
    pf.setGraph(&graph);

    graph.addEdge(0, 1, 3);
    graph.addEdge(0, 7, 8);
    graph.addEdge(1, 2, 8);
    graph.addEdge(1, 7, 11);
//    graph.addEdge(2, 3, 7);
//    graph.addEdge(2, 5, 4);
    graph.addEdge(2, 8, 2);
    graph.addEdge(3, 4, 9);
    graph.addEdge(3, 5, 13);
    graph.addEdge(4, 5, 10);
    graph.addEdge(4, 9, 1);
//    graph.addEdge(5, 6, 2);
    graph.addEdge(6, 7, 1);
    graph.addEdge(7, 8, 7);

    std::vector<unsigned int> path;

    ASSERT_FALSE(pf.findShortestPath(0, 4, path));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "auction_manager_utest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
