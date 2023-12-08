//
// Created by vini on 18/11/23.
//

#include "CityGraph.h"

#include <limits>
#include <vector>
#include <list>
#include <queue>
#include <iostream>
using namespace std;

CityGraph::CityGraph(const int numVertices) : numVertices(numVertices), numEdges(0) {
    adjLists.resize(numVertices);
}

CityGraph::~CityGraph() = default;


bool CityGraph::addEdge(const int v1, const int v2, const float distance) {
    if (v1 >= numVertices || v2 >= numVertices) return false;

    for (auto &[weight, node]: adjLists[v1]) {
        if (node == v2) return false;
    }
    adjLists[v1].emplace_back(distance,v2);
    adjLists[v2].emplace_back(distance,v1);
    numEdges++;
    return true;
}

bool CityGraph::removeEdge(const int v1, const int v2) {
    if (v1 >= numVertices || v2 >= numVertices) return false;

    for (auto &[distance, node]: adjLists[v1]) {
        if (node == v2) {
            adjLists[v1].remove({distance, node});
            adjLists[v2].remove({distance, v1});
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

vector<Deliveryman> CityGraph::getNearestDeliverymans(const Order & order) const {
    //Determines the nearest deliveryman to the store
    vector<Deliveryman> nearest;
    float *dist = new float[numVertices];
    int *parents = new int[numVertices];
    Dijkstra(order.node1, dist, parents);

    //creates a heap with the deliverymans to find the n nearest
    priority_queue<pair<float, Deliveryman>,
        vector<pair<float,Deliveryman>>, greater<>> heap;
    for (Deliveryman driver: deliverymans) {
        heap.emplace(dist[order.store], driver);
    }
    float const min_dist = dist[order.store];
    for (int i = 0;!heap.empty() && heap.top().first==min_dist ; ++i) {
        nearest.emplace_back(heap.top().second);
        heap.pop();
    }

    delete[] dist;
    delete[] parents;
    return nearest;
}

vector<int> CityGraph::getDeliveryPath(Deliveryman const & deliveryman, Order const & order) const{
    //get path of deliveryman to store and then to the client

    float *dist = new float[numVertices];
    int *parents = new int[numVertices];
    //undirected graph, so the path is the same in both ways, so we can use the same dijkstra to find the path
    //from the deliveryman to the store and from the store to the client
    Dijkstra(order.store, dist, parents);

    //get path from deliveryman to store
    vector<int> path;
    path.push_back(deliveryman.node);
    while (parents[path.back()] != order.store) {
        if (parents[path.back()] == -1) {
            return {};
        }
        path.push_back(parents[path.back()]);
    }
    if(path.back()!=order.store) path.push_back(order.store);

    vector<int> path2;
    //client addres is in a edge, so we need to check which node has the shortest path to the client
    int node = dist[order.node1] + order.distance1 < dist[order.node2] + order.distance2 ? order.node1 : order.node2;
    //get path from store to client
    path2.push_back(node);
    while (parents[path2.back()] != order.store) {
        if (parents[path2.back()] == -1) return {};
        path2.push_back(parents[path2.back()]);
    }

    for(int i=path2.size()-1; i>=0; i--)
        path.push_back(path2[i]);
    delete[] dist;
    delete[] parents;
    return path;
}

vector<tuple<Deliveryman, DistributionCenter, vector<int>>> CityGraph::getDeliveryPathWithDistribution(const Order & order) {
    int * cptCenters1 = new int[numVertices];
    float * distCenters1 = new float[numVertices];
    Dijkstra(order.node1, distCenters1, cptCenters1);
    int * cptCenters2 = new int[numVertices];
    float * distCenters2 = new float[numVertices];
    Dijkstra(order.node2, distCenters2, cptCenters2);
    adjLists.emplace_back();
    numVertices++;

    auto * nearestNodes = new pair<DistributionCenter, int>[numVertices]; //nearest distribution center to a node

    for(auto center: distributionCenters) {
        //we are adding a new node to the graph, that is connected to all distribution centers with the distance
        //from the center to the client so that we can find the cheapest path to the client passing by a center
        if (!center.products.count(order.product)&& center.products[order.product].first > 0) continue;
        float const distCenterClient = min(distCenters1[center.node] + order.distance1,
                                           distCenters2[center.node] + order.distance2);
        int const nodeClient = distCenterClient == distCenters1[center.node] + order.distance1 ?
                               order.node1 : order.node2;
        //add edge from the new node to the center and from the center to the new node
        adjLists[center.node].emplace_back(distCenterClient, numVertices-1);
        adjLists[numVertices-1].emplace_back(distCenterClient, center.node);
        nearestNodes[center.node] = make_pair(center,nodeClient);
    }

    int * cptDrivers = new int[numVertices];
    float * distDrivers = new float[numVertices];
    Dijkstra(numVertices-1, distDrivers, cptDrivers);
    //we add all deliverymans to a heap to find the ones closest to the client one
    priority_queue<pair<float, Deliveryman>,
        vector<pair<float,Deliveryman>>, greater<>> driversHeap;
    for(auto deliveryman: deliverymans) {
        driversHeap.emplace(distDrivers[deliveryman.node], deliveryman);
    }

    //get paths from deliveryman to distribution center then to client
    vector<tuple<Deliveryman, DistributionCenter, vector<int>>> paths;
    const float min_dist = distDrivers[driversHeap.top().second.node];
    paths.emplace_back(driversHeap.top().second,DistributionCenter(),vector<int>{driversHeap.top().second.node});
    //we'll get all paths with the same distance to the client
    for (int i = 0; !driversHeap.empty() && driversHeap.top().first == min_dist;) {
        if (cptDrivers[get<2>(paths[i]).back()] == -1) return {};
        //if the parent of the node is the added node, we found the path to the distribution center
        if (cptDrivers[get<2>(paths[i]).back()]==numVertices-1) {
            int const * cptCenters = nearestNodes[get<2>(paths[i]).back()].second == order.node1 ? cptCenters1 : cptCenters2;
            int const node = nearestNodes[get<2>(paths[i]).back()].second;
            //get the distribution center of the node
            get<1>(paths[i]) = nearestNodes[get<2>(paths[i]).back()].first;

            //get the path from the distribution center to the client
            while(get<2>(paths[i]).back() != node) {
                if (cptCenters[get<2>(paths[i]).back()] == -1) return {};
                get<2>(paths[i]).push_back(cptCenters[get<2>(paths[i]).back()]);
            }
            i++;
            driversHeap.pop();
            //if the next deliveryman has the same distance to the client, we add it to the paths
            if (driversHeap.top().first == min_dist)
                paths.emplace_back(driversHeap.top().second,DistributionCenter(),vector<int>{driversHeap.top().second.node});
            else break;
        }
        else
            get<2>(paths[i]).push_back(cptDrivers[get<2>(paths[i]).back()]);

    }
    //remove the new node from the graph
    for(auto edge: adjLists[numVertices-1]) {
        adjLists[edge.node].pop_back();
    }
    numVertices--;
    adjLists.pop_back();
    delete[] cptCenters1;
    delete[] cptCenters2;
    delete[] cptDrivers;
    delete[] distCenters1;
    delete[] distCenters2;
    delete[] distDrivers;
    return paths;
}

void CityGraph::printAdjLists() const {
    for(auto & adjList: adjLists) {
        for(auto const edge: adjList) {
            cout << edge.node << ","<< edge.distance << " ";
        }
        cout << endl;
    }
}


