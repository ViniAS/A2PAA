#include <iostream>
#include "./classes/CityGraph.h"
#include <chrono>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>

using namespace std;

int mainLoop();
vector<vector<int>> readCSV(const string&);

int main() {
    int return_value = mainLoop();

    return return_value;
}


int mainLoop() {
    int iSentinel = 0;
    bool bExecute = true;
    int iSize;

    cout << "Bem-vindo! Digite o tamanho do grafo que quer usar: ";
    cin >> iSize;

    CityGraph graph(iSize);

    while (bExecute) {
        cout << "\n======================================================" << endl;
        cout << "Digite a operação que deseja realizar." << endl;
        cout << "1 - Criar grafo a partir de arquivo csv com a matriz de arestas (-1 significa que não há aresta)" << endl;
        cout << "2 - Adicionar centro de distribuição" << endl;
        cout << "3 - Adicionar entregador" << endl;
        cout << "4 - Executar Operação 1" << endl;
        cout << "5 - Executar Operação 2" << endl;
        cout << "6 - Executar Operação 3" << endl;
        cout << "7 - Imprimir grafo" << endl;
        cout << "8 - Sair" << endl;

        cout << "\n>>> "; cin >> iSentinel;

        if (!iSentinel) {
            cout << "Input inválido!" << endl;

            cin.clear();
            cin.ignore(10000, 'n');

            continue;
        }

        switch (iSentinel) {
            case 1: { // Input csv
                string file;
                cout << "Digite o caminho absoluto do arquivo csv: ";
                cin >> file;
                //cout << file << endl;
                vector<vector<int>> rows = readCSV(file);

                for (int i = 0; i < rows.size(); i++) {
                    for (int j = 0; j < rows[i].size(); j++) {
                        //cout << rows[i][j] << endl;
                        if (rows[i][j] != -1) {
                            graph.addEdge(i, j, rows[i][j]);
                        }
                    }
                }

                cout << "\nEsta é a lista de adjacência do grafo:" << endl;
                graph.printAdjLists();
                break;
            }

            case 2: { // Adicionar CD
                int node;
                cout << " >> Insira o nó onde o centro de distribuição está: ";
                cin >> node;

                unordered_map<string, pair<int, float>> products;
                
                bool adding = true;
                while (adding) {
                    bool input;
                    cout << endl;
                    cout << " >> Digite 0 para sair ou 1 para adicionar um produto: ";
                    cin >> input;

                    if (input) {
                        string name;
                        int quant;
                        float weight;
                        cout << "  > Digite o nome do produto: "; cin >> name;
                        cout << "  > Digite a quantidade do produto: "; cin >> quant;
                        cout << "  > Digite o peso do produto: "; cin >> weight;

                        products[name] = make_pair(quant, weight);
                    }
                    else {
                        adding = false;
                    };
                }
                
                DistributionCenter dc(node, products);
                graph.distributionCenters.emplace_back(dc);
                cout << "\nCentro de distribuição adicionado com sucesso." << endl;
                
                break;
            }

            case 3: { // Adicionar entregador
                int node;
                float capacity;
                cout << " >> Insira o nó onde o entregador está: ";
                cin >> node;

                cout << " >> Insira a capacidade de carga do veículo do entregador: ";
                cin >> capacity;
                
                Deliveryman deliveryman(node, capacity);
                graph.deliverymen.emplace_back(deliveryman);
                cout << "\nEntregador adicionado com sucesso." << endl;
                
                break;
            }

            case 4: { // Operation 1
                // Ask the location of the client
                int node1, node2;
                int store;
                float dist1, dist2;
                string product;
                cout << "Insira os vértices de endereço do cliente." << endl;
                cout << "> "; cin >> node1;
                cout << "> "; cin >> node2;

                cout << "Insira as distâncias aos respectivos vértices." << endl;
                cout << "> "; cin >> dist1;
                cout << "> "; cin >> dist2;
                
                cout << "Insira o vértice da loja: ";
                cin >> store;

                cout << "Insira o nome do produto pedido: ";
                cin >> product;

                cout << endl;
                
                // Ask the order
                const Order order(node1, node2, dist1, dist2, product, store);
                
                // Get the start time
                auto start = std::chrono::high_resolution_clock::now();

                vector<Deliveryman> nearest = graph.getNearestDeliverymen(order);

                // Get the end time
                auto end = std::chrono::high_resolution_clock::now();
                // Calculate and print the duration
                std::chrono::duration<double> duration = end - start;

                cout << "Entregadores mais próximos: ";
                for (const auto& element : nearest) {
                    cout << element.node << " ";
                }
                cout << endl;

                cout << "Tempo de execução: " << duration.count() << " segundos." << endl;
                
                break;
            }

            case 5: { // Operation 2
                // Ask the location of the client
                int node1, node2;
                int store;
                float dist1, dist2;
                string product;
                cout << "Insira os vértices de endereço do cliente." << endl;
                cout << "> "; cin >> node1;
                cout << "> "; cin >> node2;

                cout << "Insira as distâncias aos respectivos vértices." << endl;
                cout << "> "; cin >> dist1;
                cout << "> "; cin >> dist2;

                cout << "Insira o vértice da loja: ";
                cin >> store;

                cout << "Insira o nome do produto pedido: ";
                cin >> product;

                cout << endl;

                // Ask the order
                const Order order(node1, node2, dist1, dist2, product, store);
                vector<Deliveryman> nearest = graph.getNearestDeliverymen(order);

                // Get the start time
                auto start = std::chrono::high_resolution_clock::now();

                vector<int> path = graph.getDeliveryPath(nearest[0], order);

                // Get the end time
                auto end = std::chrono::high_resolution_clock::now();
                // Calculate and print the duration
                std::chrono::duration<double> duration = end - start;
                
                cout << "Rota de entrega: ";
                for (const auto& element : path) {
                    cout << element << " ";
                }
                cout << endl;
                
                cout << "Tempo de execução: " << duration.count() << " segundos." << endl;
                
                break;
            }

            case 6: { // Operation 3
                // Ask the location of the client
                int node1, node2;
                int store;
                float dist1, dist2;
                string product;
                cout << "Insira os vértices de endereço do cliente." << endl;
                cout << "> "; cin >> node1;
                cout << "> "; cin >> node2;

                cout << "Insira as distâncias aos respectivos vértices." << endl;
                cout << "> "; cin >> dist1;
                cout << "> "; cin >> dist2;

                cout << "Insira o vértice da loja: ";
                cin >> store;

                cout << "Insira o nome do produto pedido: ";
                cin >> product;

                cout << endl;

                // Ask the order
                const Order order(node1, node2, dist1, dist2, product, store);
                
                // Get the start time
                auto start = std::chrono::high_resolution_clock::now();

                auto paths = graph.getDeliveryPathWithDistribution(order);

                // Get the end time
                auto end = std::chrono::high_resolution_clock::now();
                // Calculate and print the duration
                std::chrono::duration<double> duration = end - start;

                // Print vector elements
                for (const auto& element : paths) {
                    std::cout << "- Entregador: " << std::get<0>(element).node << ", "
                              << "CD: " << std::get<1>(element).node << ", "
                              << "Rota: ";

                    const auto& path = std::get<2>(element);
                    for (const auto& vertex : path) {
                        std::cout << vertex << " -> ";
                    }
                    std::cout << "Fim" << std::endl;
                }

                cout << "Tempo de execução: " << duration.count() << " segundos." << endl;
                
                break;
            }

            case 7: { // Print Graph
                graph.printAdjLists();
                break;
            }

            case 8: { // Quit
                bExecute = false;
                break;
            }

            default: {
                cout << "Input inválido!" << endl;
                bExecute = false;
                break;
            }
        }
    }

    cout << "\nObrigado por utilizar o programa!" << endl;
    cout << "======================================================" << endl;

    return 0;
}

vector<vector<int>> readCSV(const string& filename) {
    ifstream file;
    file.open(filename);

    string line;
    vector<vector<int>> rows;

    while (getline(file, line)) {
        stringstream ss(line);
        string cell;
        vector<int> row;

        while (getline(ss, cell, ',')) {
            row.push_back(stoi(cell));
            //cout << cell << endl;
        }

        rows.push_back(row);
    }

    return rows;
}
