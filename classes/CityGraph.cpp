//
// Created by vini on 18/11/23.
//

#include "CityGraph.h"

#include <limits>
#include <vector>
#include <list>
#include <queue>

using namespace std;

CityGraph::CityGraph(const int numVertices) : numVertices(numVertices), numEdges(0) {
    adjLists.resize(numVertices);
}

CityGraph::~CityGraph() = default;

void CityGraph::addNode() {
    numVertices++;
    adjLists.resize(numVertices);
}

bool CityGraph::addEdge(const int v1, const int v2, const float distance) {
    if (v1 >= numVertices || v2 >= numVertices) return false;

    for (auto &[weight, node]: adjLists[v1]) {
        if (node == v2) return false;
    }
    adjLists[v1].emplace_back(v2,distance);
    adjLists[v2].emplace_back(v1,distance);
    numEdges++;
    return true;
}

bool CityGraph::removeEdge(const int v1, const int v2) {
    if (v1 >= numVertices || v2 >= numVertices) return false;
    for (auto &[distance, node]: adjLists[v1]) {
        if (node == v2) {
            adjLists[v1].remove({node, distance});
            adjLists[v2].remove({v1, distance});
            numEdges--;
            return true;
        }
    }

    return false;
}

int CityGraph::getNumVertices() const {
    return numVertices;
}

int CityGraph::getNumEdges() const {
    return numEdges;
}

bool CityGraph::hasEdge(const int v1, const int v2) const {
    if (v1 >= numVertices || v2 >= numVertices) return false;
    for (auto &[weight, node]: adjLists[v1]) {
        if (node == v2) return true;
    }
    return false;
}

unsigned long CityGraph::getDegree(const int v) const {
    if (v >= numVertices) return 0;
    return adjLists[v].size();
}

vector<list<Edge>> CityGraph::getAdjLists() const {
    return adjLists;
}

void CityGraph::Dijkstra(int s, float *dist, int *parents) const {
    bool *checked = new bool[numVertices];
    for (int i = 0; i < numVertices; ++i) {
        dist[i] = numeric_limits<float>::infinity();
        parents[i] = -1;
        checked[i] = false;
    }
    dist[s] = 0;
    parents[s] = s;
    priority_queue<std::pair<float, int>,
        std::vector<std::pair<float, int>>, std::greater<>> heap;
    heap.emplace(0, s);

    while(!heap.empty()) {
        int const v = heap.top().second;
        heap.pop();
        if (dist[v] == std::numeric_limits<float>::infinity()) break;
        for(auto &[weight, node]: adjLists[v]) {
            if (!checked[node]) {
                if (dist[v] + weight < dist[node]) {
                    dist[node] = dist[v] + weight;
                    parents[node] = v;
                    heap.emplace(dist[node], node);
                }
            }
        }
        checked[v] = true;
    }
    delete[] checked;
}