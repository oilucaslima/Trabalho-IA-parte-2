#include "functions.h"
int memoryUsage = 0, memoryDFS = 0;

vector<vector<int>> matriz = {
        {0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0}
};

unordered_map<char, int> char_to_index = {
        {'A', 0}, {'B', 1}, {'C', 2}, {'D', 3}, {'E', 4},
        {'F', 5}, {'G', 6}, {'H', 7}, {'I', 8}, {'J', 9},
        {'K', 10}, {'L', 11}, {'M', 12}, {'N', 13}, {'O', 14},
        {'P', 15}, {'Q', 16}, {'R', 17}, {'S', 18}, {'T', 19},
        {'U', 20}, {'V', 21}, {'X', 22}, {'Y', 23}, {'Z', 24}
};

void DFS(vector<vector<int>>& matriz, int start, int end, vector<bool>& visited, vector<int>& parent, unordered_set<int>& currentPath, bool& foundPath) {
    if (start == end) {
        // Print the path and set the foundPath flag
        cout << "Path found: ";
        int curr = end;
        while (curr != -1) {
            cout << (char)(curr + 'A') << " ";
            curr = parent[curr];
        }
        cout << endl;
        foundPath = true;
        return;
    }

    if (visited[start] || currentPath.count(start) || foundPath) {
        return; // Avoid cycles, already visited nodes, or if a path is found
    }

    visited[start] = true;
    currentPath.insert(start);
    memoryDFS += sizeof(start);

    for (int i = 0; i < (int)matriz.size(); ++i) {
        if (matriz[start][i] && !visited[i]) {
            parent[i] = start;
            DFS(matriz, i, end, visited, parent, currentPath, foundPath);
        }
    }

    // Clean up the current path for backtracking
    currentPath.erase(start);
    visited[start] = false;
}

void call_dfs() {
    cout << " -- DFS -- " << endl;
    // Assuming 'U' has index 20 and 'E' has index 4
    int start = 20; // 'U'
    int end = 1;   // 'E'

    int n = matriz.size();
    vector<bool> visited(n, false);
    vector<int> parent(n, -1);
    unordered_set<int> currentPath;

    bool foundPath = false;
    DFS(matriz, start, end, visited, parent, currentPath, foundPath);

    cout << "Memória usada: " << memoryDFS << " B\n";
}

void BFS(vector<vector<int>>& matriz, int start, int end) {
    int n = matriz.size();
    vector<bool> visited(n, false);
    queue<int> q;
    vector<int> parent(n, -1);

    q.push(start);
    visited[start] = true;

    bool foundPath = false;

    while (!q.empty() && !foundPath) {
        int curr = q.front();
        q.pop();

        if (curr == end) {
            // Print the path
            cout << "Path found: ";
            while (curr != -1) {
                cout << (char)(curr + 'A') << " ";
                curr = parent[curr];
            }
            cout << endl;
            foundPath = true;
            break;
        }

        for (int i = 0; i < n; ++i) {
            if (matriz[curr][i] && !visited[i]) {
                visited[i] = true;
                parent[i] = curr;
                q.push(i);
                memoryUsage += sizeof(i);
            }
        }
    }
}

void call_bfs() {
    cout << " -- BFS -- " << endl;
    // Assuming 'U' has index 20 and 'E' has index 4
    int start = 20; // 'U'
    int end = 1;   // 'E'

    BFS(matriz, start, end);

    cout << "Memória usada: " << memoryUsage << " B\n";
}

// ---------------------------------------------

// Coordenadas dos nós (x, y) - grid 5x5
vector<pair<int, int>> coordenadas = {
    {0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4},
    {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4},
    {2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4},
    {3, 0}, {3, 1}, {3, 2}, {3, 3}, {3, 4},
    {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}
};

/// Função para calcular a heurística de Manhattan
int heuristicaManhattan(int atual, int destino) {
    auto [x1, y1] = coordenadas[atual];
    auto [x2, y2] = coordenadas[destino];
    return abs(x1 - x2) + abs(y1 - y2);
}

// Variáveis globais para medir o uso de memória
size_t memoryAStar = 0; // Para medir o custo de memória no A*

// Função para encontrar o menor caminho usando A*
void AStar(vector<vector<int>>& matriz, int start, int end) {
    int n = matriz.size();
    vector<int> g_cost(n, numeric_limits<int>::max()); // Custo acumulado até o nó
    vector<int> f_cost(n, numeric_limits<int>::max()); // Custo total estimado (g + heurística)
    vector<int> parent(n, -1); // Para reconstruir o caminho
    vector<bool> visited(n, false);

    // Usamos um priority_queue para gerenciar os nós a serem visitados
    auto cmp = [](pair<int, int> a, pair<int, int> b) { return a.second > b.second; };
    priority_queue<pair<int, int>, vector<pair<int, int>>, decltype(cmp)> open_set(cmp);

    g_cost[start] = 0;
    f_cost[start] = heuristicaManhattan(start, end);
    open_set.push({start, f_cost[start]});

    // Medindo a memória usada pelas variáveis do A*
    memoryAStar += sizeof(g_cost) + sizeof(f_cost) + sizeof(parent) + sizeof(visited);
    memoryAStar += sizeof(open_set); // A fila de prioridade também é considerada

    while (!open_set.empty()) {
        int current = open_set.top().first;
        open_set.pop();

        if (current == end) {
            // Reconstruir o caminho
            cout << "Path found: ";
            int temp = end;
            while (temp != -1) {
                cout << (char)(temp + 'A') << " ";
                temp = parent[temp];
            }
            cout << endl;
            //cout << "Custo total: " << g_cost[end] << endl;
            cout << "Memória usada: " << memoryAStar << " B" << endl;
            return;
        }

        visited[current] = true;

        for (int neighbor = 0; neighbor < n; ++neighbor) {
            if (matriz[current][neighbor] == 0 || visited[neighbor])
                continue;

            int tentative_g_cost = g_cost[current] + matriz[current][neighbor];
            if (tentative_g_cost < g_cost[neighbor]) {
                parent[neighbor] = current;
                g_cost[neighbor] = tentative_g_cost;
                f_cost[neighbor] = g_cost[neighbor] + heuristicaManhattan(neighbor, end);
                open_set.push({neighbor, f_cost[neighbor]});
                
                // Medindo o uso de memória para cada nó processado
                memoryAStar += sizeof(neighbor) + sizeof(f_cost[neighbor]) + sizeof(g_cost[neighbor]);
            }
        }
    }

    cout << "No path found from " << (char)(start + 'A') << " to " << (char)(end + 'A') << endl;
}

// Função para chamar a busca A*
void call_astar() {
    cout << "\n -- A* com Heurística de Manhattan -- " << endl;
    int start = 20; // 'U'
    int end = 1;   // 'E'

    AStar(matriz, start, end);
}
