#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <random>
#include <cstdlib>
#include <chrono>
#include "charts.h"
#include "../classes/CityGraph.h"

using namespace std;

void exportToCSV(const vector<vector<double>>&, const string&);
CityGraph createCityGraph(int);
void populateCityGraph(CityGraph&);
void operation1();
void operation2();
void operation3();

int chartMaker() {
    operation1();
    operation2();
    operation3();
    return 0;
}

void exportToCSV(const std::vector<std::vector<double>>& data, const std::string& filename) {
    std::ofstream file(filename);

    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i != row.size() - 1)
                file << ",";
        }
        file << "\n";
    }

    file.close();
}

CityGraph createCityGraph(int numVertices) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(1.0, 100.0);

    CityGraph graph(numVertices);

    // Create an edge between each pair of consecutive nodes to ensure connectivity
    for (int i = 0; i < numVertices - 1; i++) {
        float weight = dis(gen);
        graph.addEdge(i, i + 1, weight);
    }

    // Create an edge between the last and the first node to ensure connectivity
    float weight = dis(gen);
    graph.addEdge(numVertices - 1, 0, weight);

    // Add additional edges
    int maxAdditionalEdgesPerVertex = 5;
    for (int j = 0; j < numVertices; j++) {
        for (int k = 0; k < maxAdditionalEdgesPerVertex; k++) {
            int node1 = j;
            int node2 = (j + k) % numVertices;
            weight = dis(gen);

            if (node1 != node2) {
                graph.addEdge(node1, node2, weight);
            }
        }
    }
    
    return graph;
}


void populateCityGraph(CityGraph& graph) {
    int numVertices = graph.getNumVertices();
    int numDeliverymen = numVertices / 100 + 1;
    int numDistributionCenters = numVertices / 100 + 1;

    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> distrib(0.0, 1.0);

    // Add deliverymen
    for (int i = 0; i < numDeliverymen; i++) {
        int location = int(distrib(gen) * numVertices) % numVertices;
        graph.deliverymen.emplace_back(location, 10);
    }

    // Add distribution centers
    unordered_map<string, pair<int,float>> products = {{"Product1", {10, 1.0f}}, {"Livro", {5, 0.5f}}};
    for (int i = 0; i < numDistributionCenters; i++) {
        int location = int(distrib(gen) * numVertices) % numVertices;
        graph.distributionCenters.emplace_back(location, products);
    }
}

void operation1() {
    vector<vector<double>> operation1Matrix;

    for (int i = 10; i <= 100000; i+=1000) {
        CityGraph graph = createCityGraph(i);
        populateCityGraph(graph);

        std::string product = "Livro";

        const Order exampleOrder(0, 1, 0.0f, 1.0f, product, 0);

        // Get the start time
        auto start = std::chrono::high_resolution_clock::now();

        vector<Deliveryman> nearest = graph.getNearestDeliverymen(exampleOrder);

        // Get the end time
        auto end = std::chrono::high_resolution_clock::now();
        // Calculate and print the duration
        std::chrono::duration<double> duration = end - start;

        operation1Matrix.push_back({static_cast<double>(i), duration.count() * 1000});

        cout << i << endl;
    }

    exportToCSV(operation1Matrix, "../times/operation1.csv");
    cout << "Operation 1 done" << endl;
}

void operation2() {
    vector<vector<double>> operation1Matrix;

    for (int i = 10; i <= 100000; i+=1000) {
        CityGraph graph = createCityGraph(i);
        populateCityGraph(graph);

        std::string product = "Livro";

        const Order order(0, 1, 0.0f, 1.0f, product, 0);
        vector<Deliveryman> nearest = graph.getNearestDeliverymen(order);

        // Get the start time
        auto start = std::chrono::high_resolution_clock::now();

        vector<int> path = graph.getDeliveryPath(nearest[0], order);

        // Get the end time
        auto end = std::chrono::high_resolution_clock::now();
        // Calculate and print the duration
        std::chrono::duration<double> duration = end - start;

        operation1Matrix.push_back({static_cast<double>(i), duration.count() * 1000});

        cout << i << endl;
    }

    exportToCSV(operation1Matrix, "../times/operation2.csv");
    cout << "Operation 2 done" << endl;
}

void operation3() {
    vector<vector<double>> operation1Matrix;

    for (int i = 10; i <= 100000; i+=1000) {
        CityGraph graph = createCityGraph(i);
        populateCityGraph(graph);

        std::string product = "Livro";

        const Order order(0, 1, 0.0f, 1.0f, product, 0);
        vector<Deliveryman> nearest = graph.getNearestDeliverymen(order);

        // Get the start time
        auto start = std::chrono::high_resolution_clock::now();

        auto paths = graph.getDeliveryPathWithDistribution(order);

        // Get the end time
        auto end = std::chrono::high_resolution_clock::now();
        // Calculate and print the duration
        std::chrono::duration<double> duration = end - start;

        operation1Matrix.push_back({static_cast<double>(i), duration.count() * 1000});

        cout << i << endl;
    }

    exportToCSV(operation1Matrix, "../times/operation3.csv");
    cout << "Operation 3 done" << endl;
}