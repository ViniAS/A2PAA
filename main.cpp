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

    cout << "Bem-vindo! Digite o tamanho do grafo que quer usar:" << endl;
    cin >> iSize;

    CityGraph graph(iSize);

    while (bExecute) {
        cout << "Digite a operação que deseja realizar:" << endl;
        cout << "1 - Criar grafo a partir de arquivo csv com a matriz de arestas (-1 significa que não há aresta)" << endl;
        cout << "2 - Executar Operação 1" << endl;
        cout << "3 - Executar Operação 2" << endl;
        cout << "4 - Executar Operação 3" << endl;
        cout << "5 - Imprimir grafo" << endl;
        cout << "6 - Sair" << endl;

        cin >> iSentinel;

        if (!iSentinel) {
            cout << "Input inválido!" << endl;

            cin.clear();
            cin.ignore(10000, 'n');

            continue;
        }

        switch (iSentinel) {
            case 1: { // Input csv
                string file;
                cout << "Digite o nome do arquivo csv: ";
                cin >> file;
                cout << file << endl;
                vector<vector<int>> rows = readCSV(file);

                for (int i = 0; i < rows.size(); i++) {
                    for (int j = 0; j < rows[i].size(); j++) {
                        cout << rows[i][j] << endl;
                        if (rows[i][j] != -1) {
                            graph.addEdge(i, j, rows[i][j]);
                        }
                    }
                }
                graph.printAdjLists();
                break;
            }

            case 2: { // Operation 1
                continue;
            }

            case 3: { // Operation 2
                continue;
            }

            case 4: { // Operation 3
                continue;
            }

            case 5: { // Print Graph
                graph.printAdjLists();
                continue;
            }

            case 6: { // Quit
                bExecute = false;
                break;
            }

            default: {
                cout << "Input inválido!" << endl;
                break;
            }
        }
    }

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
            cout << cell << endl;
        }

        rows.push_back(row);
    }

    return rows;
}