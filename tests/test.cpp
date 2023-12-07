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

TEST(CityGraphTest, AddEdgeReturnValues) {
    CityGraph graph(5);

    // Test adding a valid edge
    EXPECT_TRUE(graph.addEdge(0, 1, 1.0f));

    // Test adding the same edge again
    EXPECT_FALSE(graph.addEdge(0, 1, 1.0f));

    // Test adding an edge with a non-existent vertex
    EXPECT_FALSE(graph.addEdge(0, 5, 1.0f));
}

TEST(CityGraph, RemoveEdge) {
    CityGraph g(5);
    g.addEdge(0, 1, 1.0);
    g.removeEdge(0, 1);
    EXPECT_FALSE(g.hasEdge(0, 1));
}

TEST(CityGraphTest, RemoveEdgeReturnValues) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 1.0f);

    // Test removing an existing edge
    EXPECT_TRUE(graph.removeEdge(0, 1));

    // Test removing the same edge again
    EXPECT_FALSE(graph.removeEdge(0, 1));

    // Test removing an edge with a non-existent vertex
    EXPECT_FALSE(graph.removeEdge(0, 5));
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

TEST(CityGraphTest, HasEdge) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 1.0f);

    // Test checking an existing edge
    EXPECT_TRUE(graph.hasEdge(0, 1));

    // Test checking a non-existing edge
    EXPECT_FALSE(graph.hasEdge(0, 2));

    // Test checking an edge with a non-existent vertex
    EXPECT_FALSE(graph.hasEdge(0, 5));
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

TEST(CityGraphTest, DijkstraDistances) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);

    float dist[5];
    int parents[5];

    graph.Dijkstra(0, dist, parents);

    // Test distances
    EXPECT_FLOAT_EQ(dist[0], 0.0f);
    EXPECT_FLOAT_EQ(dist[1], 1.0f);
    EXPECT_FLOAT_EQ(dist[2], 3.0f);
    EXPECT_FLOAT_EQ(dist[3], 6.0f);
    EXPECT_FLOAT_EQ(dist[4], 10.0f);

    // Test parents
    EXPECT_EQ(parents[0], 0);
    EXPECT_EQ(parents[1], 0);
    EXPECT_EQ(parents[2], 1);
    EXPECT_EQ(parents[3], 2);
    EXPECT_EQ(parents[4], 3);
}

TEST(CityGraphTest, GetNearestDeliverymans) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);

    Deliveryman d1(0, 10);
    Deliveryman d2(1, 10);
    Deliveryman d3(2, 10);

    std::string product = "Livro";
    // Order exampleOrder(1, 2, 1.5f, 2.5f, product, 0);
    // std::cout << d1.node << std::endl;
    // Deliveryman* nearest = graph.getNearestDeliverymans(exampleOrder, 1);

    // std::cout << nearest[0].node << std::endl;
    // // Test nearest deliverymans
    // EXPECT_EQ(nearest[0].node, d1.node);

    // delete[] nearest;
}

TEST(CityGraphTest, GetDeliveryPathLinearCase) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);

    Deliveryman deliveryman(0, 100);

    std::string product = "Livro";
    Order order(3, 4, 1.5f, 2.5f, product, 0);

    vector<int> path = graph.getDeliveryPath(deliveryman, order);

    // Test delivery path
    vector<int> expectedPath = {0, 1, 2, 3};
    EXPECT_EQ(path, expectedPath);
}

TEST(CityGraphTest, GetDeliveryPathNonTrivialCase) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);
    

    Deliveryman deliveryman(0, 100);

    std::string product = "Livro";
    Order order(3, 4, 1.5f, 2.5f, product, 1);

    vector<int> path = graph.getDeliveryPath(deliveryman, order);

    // Test delivery path
    vector<int> expectedPath = {0, 1, 2, 3};
    EXPECT_EQ(path, expectedPath);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}