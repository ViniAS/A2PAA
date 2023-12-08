#include <iostream>
#include <gtest/gtest.h>
#include "../classes/CityGraph.h"

#include <list>

using namespace std;

TEST(CityGraph, HasEdgeNaive) {
    CityGraph g(5);
    EXPECT_FALSE(g.hasEdge(0, 1));
}

TEST(CityGraph, AddEdge) {
    CityGraph g(3);
    g.addEdge(0, 1, 1.0);
    EXPECT_TRUE(g.hasEdge(0, 1));
    EXPECT_TRUE(g.hasEdge(1, 0));

    EXPECT_FALSE(g.hasEdge(1, 2));
    EXPECT_FALSE(g.hasEdge(0, 2));
    EXPECT_FALSE(g.hasEdge(2, 1));
    EXPECT_FALSE(g.hasEdge(2, 0));
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

    graph.deliverymans.emplace_back(0, 10);

    std::string product = "Livro";
    const Order exampleOrder(1, 2, 1.5f, 2.5f, product, 0);

    const vector<Deliveryman> nearest = graph.getNearestDeliverymans(exampleOrder);

    // Test nearest deliverymans
    EXPECT_EQ(nearest[0].node, graph.deliverymans[0].node);


}

TEST(CityGraphTest, GetNearestDeliverymansMoreThanOne) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);

    graph.deliverymans.emplace_back(0, 10);
    graph.deliverymans.emplace_back(1, 10);
    graph.deliverymans.emplace_back(2, 10);

    std::string product = "Livro";
    const Order exampleOrder(1, 2, 1.5f, 2.5f, product, 0);

    const vector<Deliveryman> nearest = graph.getNearestDeliverymans(exampleOrder);

    // Test nearest deliverymans
    EXPECT_EQ(nearest[0], graph.deliverymans[0]);


}

TEST(CityGraphTest, GetDeliveryPathLinear) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);


    graph.deliverymans.emplace_back(0, 100);

    std::string product = "Livro";
    Order order(3, 4, 1.5f, 2.5f, product, 0);

    const vector<Deliveryman> nearest = graph.getNearestDeliverymans(order);

    vector<int> path = graph.getDeliveryPath(nearest[0], order);

    // Test delivery path
    vector<int> expectedPath = {0, 1, 2, 3};
    EXPECT_EQ(path, expectedPath);
}

TEST(CityGraphTest, GetDeliveryPathNonTrivial) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);

    graph.deliverymans.emplace_back(0, 100);

    std::string product = "Livro";
    Order order(3, 4, 1.5f, 2.5f, product, 1);

    const vector<Deliveryman> nearest = graph.getNearestDeliverymans(order);

    vector<int> path = graph.getDeliveryPath(nearest[0], order);

    // Test delivery path
    vector<int> expectedPath = {0, 1, 2, 3};
    EXPECT_EQ(path, expectedPath);
}

TEST(CityGraphTest, GetDeliveryPathGoingBack) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);

    graph.deliverymans.emplace_back(1, 100);

    std::string product = "Livro";
    Order order(3, 4, 1.5f, 2.5f, product, 0);

    const vector<Deliveryman> nearest = graph.getNearestDeliverymans(order);

    vector<int> path = graph.getDeliveryPath(nearest[0], order);

    // Test delivery path
    vector<int> expectedPath = {1, 0, 1, 2, 3};
    EXPECT_EQ(path, expectedPath);
}

TEST(CityGraphTest, GetDeliveryPathClique) {
    CityGraph graph(4);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(0, 2, 1.0f);
    graph.addEdge(0, 3, 1.0f);
    graph.addEdge(1, 2, 1.0f);
    graph.addEdge(1, 3, 1.0f);
    graph.addEdge(2, 3, 1.0f);

    graph.deliverymans.emplace_back(0, 100);

    std::string product = "Livro";
    Order order(1, 2, 2.0f, 0.0f, product, 3);

    const vector<Deliveryman> nearest = graph.getNearestDeliverymans(order);

    vector<int> path = graph.getDeliveryPath(nearest[0], order);

    // Test delivery path
    vector<int> expectedPath = {0, 3, 2};
    EXPECT_EQ(path, expectedPath);
}

TEST(CityGraphTest, GetDeliveryPathHeavyWeights) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 100.0f);
    graph.addEdge(0, 2, 100.0f);
    graph.addEdge(0, 3, 1.0f);
    graph.addEdge(0, 4, 100.0f);
    graph.addEdge(1, 2, 1.0f);
    graph.addEdge(1, 3, 100.0f);
    graph.addEdge(1, 4, 1.0f);
    graph.addEdge(2, 3, 1.0f);
    graph.addEdge(2, 4, 100.0f);
    graph.addEdge(3, 4, 1.0f);

    graph.deliverymans.emplace_back(0, 100);

    std::string product = "Livro";
    Order order(1, 2, 1.0f, 0.0f, product, 4);

    const vector<Deliveryman> nearest = graph.getNearestDeliverymans(order);

    vector<int> path = graph.getDeliveryPath(nearest[0], order);

    // Test delivery path
    vector<int> expectedPath = {0, 3, 4, 1, 2};
    EXPECT_EQ(path, expectedPath);
}

TEST(CityGraphTest, GetDeliveryPathWithDistributionLinear) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);

    graph.deliverymans.emplace_back(0, 100);
    graph.deliverymans.emplace_back(1, 100);
    graph.deliverymans.emplace_back(2, 100);
    graph.deliverymans.emplace_back(3, 100);

    unordered_map<string, pair<int,float>> products = {{"Product1", {10, 1.0f}}, {"Livro", {5, 0.5f}}};

    graph.distributionCenters.emplace_back(2, products);

    std::string product = "Livro";
    Order order(3, 4, 1.5f, 2.5f, product, 0);

    auto paths = graph.getDeliveryPathWithDistribution(order);

    // Test delivery path
    EXPECT_EQ(get<0>(paths[0]).node, 2);
    EXPECT_EQ(get<1>(paths[0]).node, 2);
    vector<int> path = {2, 3};
    EXPECT_EQ(get<2>(paths[0]), path);
}

TEST(CityGraphTest, GetDeliveryPathWithDistributionGoingBack) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);

    graph.deliverymans.emplace_back(3, 100);

    unordered_map<string, pair<int,float>> products = {{"Product1", {10, 1.0f}}, {"Livro", {5, 0.5f}}};

    graph.distributionCenters.emplace_back(0, products);

    std::string product = "Livro";
    Order order(3, 4, 0.0f, 4.0f, product, 0);

    auto paths = graph.getDeliveryPathWithDistribution(order);

    // Test delivery path;
    EXPECT_EQ(get<0>(paths[0]).node, 3);
    EXPECT_EQ(get<1>(paths[0]).node, 0);
    vector<int> path = {3, 2, 1, 0, 1, 2, 3};
    EXPECT_EQ(get<2>(paths[0]), path);
}

TEST(CityGraphTest, GetDeliveryPathWithDistributionMoreThanOneCenter) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(1, 2, 2.0f);
    graph.addEdge(2, 3, 3.0f);
    graph.addEdge(3, 4, 4.0f);

    graph.deliverymans.emplace_back(2, 100);

    unordered_map<string, pair<int,float>> products = {{"Product1", {10, 1.0f}}, {"Livro", {5, 0.5f}}};

    graph.distributionCenters.emplace_back(1, products);
    graph.distributionCenters.emplace_back(0, products);

    std::string product = "Livro";
    Order order(3, 4, 0.0f, 4.0f, product, 0);
    auto paths = graph.getDeliveryPathWithDistribution(order);
    // Test delivery path
    EXPECT_EQ(get<0>(paths[0]).node, 2);
    EXPECT_EQ(get<1>(paths[0]).node, 1);
    vector<int> path = {2, 1, 2, 3};
    EXPECT_EQ(get<2>(paths[0]), path);
}

TEST(CityGraphTest, GetDeliveryPathWithDistributionClique) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 1.0f);
    graph.addEdge(0, 2, 1.0f);
    graph.addEdge(0, 3, 1.0f);
    graph.addEdge(1, 2, 1.0f);
    graph.addEdge(1, 3, 1.0f);
    graph.addEdge(2, 3, 1.0f);
    graph.addEdge(3,4,4.0f);

    graph.deliverymans.emplace_back(2, 100);

    unordered_map<string, pair<int,float>> products = {{"Product1", {10, 1.0f}}, {"Livro", {5, 0.5f}}};

    graph.distributionCenters.emplace_back(1, products);

    std::string product = "Livro";
    Order order(3, 4, 0.0f, 4.0f, product, 0);
    auto paths = graph.getDeliveryPathWithDistribution(order);
    // Test delivery path
    EXPECT_EQ(get<0>(paths[0]).node, 2);
    EXPECT_EQ(get<1>(paths[0]).node, 1);
    vector<int> path = {2, 1, 3};
    EXPECT_EQ(get<2>(paths[0]), path);
}

TEST(CityGraphTest, GetDeliveryPathWithDistributionHeavyWeights) {
    CityGraph graph(5);
    graph.addEdge(0, 1, 100.0f);
    graph.addEdge(0, 2, 100.0f);
    graph.addEdge(0, 3, 1.0f);
    graph.addEdge(0, 4, 100.0f);
    graph.addEdge(1, 2, 1.0f);
    graph.addEdge(1, 3, 100.0f);
    graph.addEdge(1, 4, 1.0f);
    graph.addEdge(2, 3, 1.0f);
    graph.addEdge(2, 4, 100.0f);
    graph.addEdge(3, 4, 1.0f);

    graph.deliverymans.emplace_back(0, 100);

    unordered_map<string, pair<int,float>> products = {{"Product1", {10, 1.0f}}, {"Livro", {5, 0.5f}}};

    graph.distributionCenters.emplace_back(4, products);

    std::string product = "Livro";
    Order order(2, 4, 0.0f, 100.0f, product, 0);

    auto paths = graph.getDeliveryPathWithDistribution(order);

    // Test delivery path
    EXPECT_EQ(get<0>(paths[0]).node, 0);
    EXPECT_EQ(get<1>(paths[0]).node, 4);
    vector<int> path = {0, 3, 4, 1, 2};
    EXPECT_EQ(get<2>(paths[0]), path);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}