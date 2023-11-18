//
// Created by vini on 18/11/23.
//

#ifndef CITYGRAPH_H
#define CITYGRAPH_H
#include <list>
#include <vector>
using namespace std;

struct Edge {
    float distance;
    int node;
    Edge(const int node, const float distance) : node(node), distance(distance) {}
    bool operator==(const Edge& other) const {
        return node == other.node && distance == other.distance;
    }
};

class CityGraph {
private:
    int numVertices;
    int numEdges;

    vector<list<Edge>> adjLists;

public:
    explicit CityGraph(int numVertices);

    ~CityGraph();

    void addNode();

    bool addEdge(int v1, int v2, float distance);

    bool removeEdge(int v1, int v2);

    [[nodiscard]] int getNumVertices() const;

    [[nodiscard]] int getNumEdges() const;

    [[nodiscard]] bool hasEdge(int v1, int v2) const;

    [[nodiscard]] unsigned long getDegree(int v) const;

    [[nodiscard]] vector<list<Edge>> getAdjLists() const;

    void Dijkstra(int s, float *dist, int *parents) const;
};



#endif //CITYGRAPH_H
