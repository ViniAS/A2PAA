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
            adjLists[v1].remove({distance, node});
            adjLists[v2].remove({distance, node});
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

Deliveryman * CityGraph::getNearestDeliverymans(const Order & order, const int n) const {
    //Determines the nearest deliveryman to the store
    auto *nearest = new Deliveryman[n];
    float *dist = new float[numVertices];
    int *parents = new int[numVertices];
    Dijkstra(order.node1, dist, parents);

    //creates a heap with the deliverymans to find the n nearest
    priority_queue<pair<float, Deliveryman>,
        vector<pair<float,Deliveryman>>, greater<>> heap;
    for (Deliveryman driver: deliverymans) {
        heap.emplace(dist[order.store], driver);
    }

    for (int i = 0; i < n; ++i) {
        nearest[i] = heap.top().second;
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
        if (parents[path.back()] == -1) return {};
        path.push_back(parents[path.back()]);
    }


    vector<int> path2;
    //client addres is in a edge, so we need to check which node has the shortest path to the client
    int node = dist[order.node1] + order.distance1 < dist[order.node2] + order.distance2 ? order.node1 : order.node2;
    //get path from store to client
    path2.push_back(node);
    while (parents[path2.back()] != order.store) {
        if (parents[path2.back()] == -1) return {};
        path2.push_back(parents[path2.back()]);
    }

    for(int i=path2.size()-1; i>0; i--)
        path.push_back(path2[i]);
    delete[] dist;
    delete[] parents;
    return path;
}

vector<int> CityGraph::getDeliveryPathWithDistribution(const Order & order){
    //tuple stores the distance from the deliveryman to the store, the deliveryman, the distribution center and the node of the client
    tuple<float, Deliveryman, DistributionCenter, int> nearest = make_tuple(numeric_limits<float>::infinity(),
        Deliveryman(), DistributionCenter(),-1);
    //we need to check all distribution centers to find the one with the cheapest path that has the product
    for(auto center: distributionCenters) {
        if (!center.products.count(order.product)&& center.products[order.product].first > 0) continue;

        //We store the cpt for future use
        if (center.cpt.empty()) {
            vector<float> dist(numVertices);
            vector<int> parents(numVertices);
            Dijkstra(center.node, dist.data(), parents.data());
            center.cpt = parents;
            center.dist = dist;
        }
        float const distCenterClient = min(center.dist[order.node1] + order.distance1,
            center.dist[order.node2] + order.distance2);
        int const nodeClient = distCenterClient == center.dist[order.node1] + order.distance1 ?
                                order.node1 : order.node2;
        //we need to check all deliverymans to find the one with the cheapest path to the distribution center
        for (auto deliveryman: deliverymans)
            if(distCenterClient+center.dist[deliveryman.node]<get<0>(nearest))
                nearest = make_tuple(distCenterClient+center.dist[deliveryman.node], deliveryman, center, nodeClient);
    }
    if (get<3>(nearest) == -1) return {};
    //get path from deliveryman to distribution center
    vector<int> path;
    path.push_back(get<1>.node);
    while (get<2>(nearest).cpt[path.back()] != get<2>(nearest).node) {
        if (get<2>(nearest).cpt[path.back()] == -1) return {};
        path.push_back(get<2>(nearest).cpt[path.back()]);
    }
    //get path from distribution center to client
    vector<int> path2;
    path2.push_back(get<3>(nearest));
    while (get<2>(nearest).cpt[path2.back()] != get<2>(nearest)) {
        if (get<2>(nearest).cpt[path2.back()] == -1) return {};
        path2.push_back(get<2>(nearest).cpt[path2.back()]);
    }

    for(int i=path2.size()-1; i>0; i--)
        path.push_back(path2[i]);

    return path;
}

vector<int> CityGraph::getDeliveryPathWithDistribution2(const Order & order) {
    int * cptCenters1 = new int[numVertices];
    float * distCenters1 = new float[numVertices];
    Dijkstra(order.node1, distCenters1, cptCenters1);
    int * cptCenters2 = new int[numVertices];
    float * distCenters2 = new float[numVertices];
    Dijkstra(order.node2, distCenters2, cptCenters2);
    adjLists.emplace_back();
    numVertices++;
    unordered_map<int, int> nearestNodes; //stores wich side of the edge has the dearest distance to the client
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
        nearestNodes[center.node] = nodeClient;
    }
    int * cptDrivers = new int[numVertices];
    float * distDrivers = new float[numVertices];
    Dijkstra(numVertices-1, distDrivers, cptDrivers);
    //we need to check all deliverymans to find the one with the cheapest path to the client
    pair<float, Deliveryman> nearestDriver = make_pair(numeric_limits<float>::infinity(), Deliveryman());
    for(auto deliveryman: deliverymans) {
        if(distDrivers[deliveryman.node]<nearestDriver.first)
            nearestDriver = make_pair(distDrivers[deliveryman.node], deliveryman);
    }
    if (nearestDriver.second.node == -1) return {};
    //get path from deliveryman to distribution center then to client
    vector<int> path;
    path.push_back(nearestDriver.second.node);
    while (true) {
        if (cptDrivers[path.back()] == -1) return {};
        path.push_back(cptDrivers[path.back()]);
        if (cptDrivers[path.back()]==numVertices-1) {
            int const * cptCenters = nearestNodes[path.back] == order.node1 ? cptCenters1 : cptCenters2;
            int const node = nearestNodes[path.back];
            while(cptCenters[path.back()] != node) {
                if (cptCenters[path.back()] == -1) return {};
                path.push_back(cptCenters[path.back()]);
            }
            break;
        }
    }
    return path;
}

