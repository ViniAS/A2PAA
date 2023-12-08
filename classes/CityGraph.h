//
// Created by vini on 18/11/23.
//

#ifndef CITYGRAPH_H
#define CITYGRAPH_H
#include <list>
#include <utility>
#include <vector>
#include <string>
#include <tuple>
#include <unordered_map>
using namespace std;

struct Edge {
    float distance;
    int node;

    Edge(const float distance, const int node) : distance(distance), node(node) {}
    bool operator==(const Edge& other) const {
        return node == other.node && distance == other.distance;
    }
};

struct Order {
    //enderecos de entrega ficam em uma aresta. node1 e node2 são os vértices da aresta.
    int node1;
    int node2;
    //distancia do endereço de entrega até o vértice node1 e node2, respectivamente
    float distance1;
    float distance2;
    string product; //produto a ser entregue
    int store; //vértice da loja
    Order(const int node1, const int node2, const float distance1, const float distance2, string & product, const int store) :
    node1(node1), node2(node2), distance1(distance1), distance2(distance2), product(std::move(product)), store(store) {}
};

struct DistributionCenter {
    int node;
    unordered_map<string, pair<int,float>> products; //product, quantity, weight
    vector<int> cpt; //Cheapest path tree
    vector<float> dist;
    DistributionCenter(const int node, unordered_map<string,pair<int,float>> & produtos) : node(node), products(std::move(produtos)) {}
    bool operator>(const DistributionCenter& other) const {
        return node > other.node;
    }
    DistributionCenter() : node(-1){}
    bool operator<(const DistributionCenter& other) const {
        return node < other.node;
    }
    bool operator==(const DistributionCenter& other) const {
        return node == other.node;
    }
};

struct Deliveryman {
    int node;
    float capacity;
    Deliveryman(const int node, const float capacity) : node(node), capacity(capacity) {}
    Deliveryman(): node(-1), capacity(-1){}
    bool operator>(const Deliveryman& other) const {
        return capacity > other.capacity;
    }

    bool operator<(const Deliveryman& other) const {
        return capacity < other.capacity;
    }

    bool operator==(const Deliveryman& other) const {
        return node == other.node && capacity == other.capacity;
    }

};
class CityGraph {
private:
    int numVertices;
    int numEdges;
    vector<list<Edge>> adjLists;

    //helper function for getDeliveryPathWithDistribution
    pair<DistributionCenter*,int>* getDistanceDistributionCenterToClient(const Order &order, const int *cptCenters1,
                                                                       const int *cptCenters2, const float *distCenters1,
                                                                          const float *distCenters2);

public:
    vector<Deliveryman> deliverymen;
    vector<DistributionCenter> distributionCenters;

    explicit CityGraph(int numVertices);

    ~CityGraph();
    
    bool addEdge(int v1, int v2, float distance);

    bool removeEdge(int v1, int v2);

    void printAdjLists() const;

    [[nodiscard]] int getNumVertices() const;

    [[nodiscard]] int getNumEdges() const;

    [[nodiscard]] bool hasEdge(int v1, int v2) const;

    [[nodiscard]] unsigned long getDegree(int v) const;

    [[nodiscard]] vector<list<Edge>> getAdjLists() const;

    void Dijkstra(int s, float *dist, int *parents) const;

    [[nodiscard]] vector<Deliveryman> getNearestDeliverymen(const Order & order);

    [[nodiscard]] vector<int> getDeliveryPath(const Deliveryman & deliveryman, const Order & order) const;

    [[nodiscard]] vector<tuple<Deliveryman, DistributionCenter, vector<int>>> getDeliveryPathWithDistribution(const Order & order);
};



#endif //CITYGRAPH_H
