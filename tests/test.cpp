#include <iostream>
#include <gtest/gtest.h>
#include "../classes/CityGraph.h"

#include <list>

using namespace std;

TEST(CityGraph, HasEdge) {
    CityGraph g(5);
    EXPECT_FALSE(g.hasEdge(0, 1));
}

TEST(CityGraph, AddEdge) {
    CityGraph g(5);
    g.addEdge(0, 1, 1.0);
    EXPECT_TRUE(g.hasEdge(0, 1));
}

TEST(CityGraph, RemoveEdge) {
    CityGraph g(5);
    g.addEdge(0, 1, 1.0);
    g.removeEdge(0, 1);
    EXPECT_FALSE(g.hasEdge(0, 1));
}

TEST(CityGraph, GetNumVertices) {
    CityGraph g(5);
    EXPECT_EQ(g.getNumVertices(), 5);
}

TEST(CityGraph, GetNumEdges) {
    CityGraph g(5);
    g.addEdge(0, 1, 1.0);
    EXPECT_EQ(g.getNumEdges(), 1);
}

TEST(CityGraph, GetDegree) {
    CityGraph g(5);
    g.addEdge(0, 1, 1.0);
    EXPECT_EQ(g.getDegree(0), 1);
}

TEST(CityGraph, GetAdjLists) {
    CityGraph g(5);
    g.addEdge(0, 1, 1.0);
    g.addEdge(0, 2, 1.0);
    g.addEdge(0, 3, 1.0);
    g.addEdge(0, 4, 1.0);
    vector<list<Edge>> adjLists = g.getAdjLists();
    EXPECT_EQ(adjLists[0].size(), 4);
}

TEST(CityGraph, Dijkstra){
    CityGraph g(5);
    g.addEdge(0, 1, 1);
    g.addEdge(0, 2, 2);
    g.addEdge(1, 3, 3);
    g.addEdge(2, 4, 4);
    float distance[5];
    int parent[5];
    g.Dijkstra(0, distance, parent);
    float expectedDistance[5] = {0, 1, 2, 4, 6};
    int expectedParent[5] = {0, 0, 0, 1, 2};
    for (int i = 0; i < 5; i++) {
        EXPECT_EQ(distance[i], expectedDistance[i]);
        EXPECT_EQ(parent[i], expectedParent[i]);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}