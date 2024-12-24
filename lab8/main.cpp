#include <iostream>

#include <fstream>
#include <sstream>

#include <vector>
#include <string>

#include <queue>
#include <stack>
#include <algorithm>
#include <unordered_map>

using namespace std;

struct Edge {
    int id1;
    int id2;
    int id3;
    double weight2;
    double weight3;
};

vector<Edge> readGraph(const string& filename) {
    ifstream infile(filename);
    if (!infile) {
        cerr << "Error opening file: " << filename << endl;
        exit(1);
    }

    vector<Edge> graph;
    string line;

    while (getline(infile, line)) {
        if (line.empty()) continue;

        istringstream iss(line);
        Edge edge;
        char delimiter;
        iss >> edge.id1 >> delimiter
            >> edge.id2 >> delimiter
            >> edge.weight2 >> delimiter
            >> edge.id3 >> delimiter
            >> edge.weight3 >> delimiter;

        graph.push_back(edge);
    }

    return graph;
}

// Построение графа в виде списка смежности
vector<vector<pair<int, double>>> buildGraph(int n, vector<Edge>& edges) {
    vector<vector<pair<int, double>>> gr(n);
    for (int i = 0; i < edges.size(); i++) {
        //cout << i << endl;
        //i = 316152;
        if (edges[i].id2 != -1) {
            gr[edges[i].id1 - 1].push_back(make_pair(edges[i].id2 - 1, edges[i].weight2));
        }
        if (edges[i].id3 != -1) {
            gr[edges[i].id1 - 1].push_back(make_pair(edges[i].id3 - 1, edges[i].weight3));
        }
    }

    return gr;
}

void makeGraphUndirected(vector<vector<pair<int, double>>>& gr) {
    for (int i = 0; i < gr.size(); i++) {
        for (int j = 0; j < gr[i].size(); j++) {
            int to = gr[i][j].first;
            double weight = gr[i][j].second;

            // Проверяем, есть ли обратное ребро из "to" в "i"
            bool found = false;
            for (int k = 0; k < gr[to].size(); k++) {
                if (gr[to][k].first == i) {
                    found = true;
                    break;
                }
            }

            // Если обратного ребра нет, добавляем его
            if (!found) {
                gr[to].push_back(make_pair(i, weight));
            }
        }
    }
}

// BFS: Поиск кратчайшего пути (без учета весов)
vector<int> bfs(int n, int s, int f, vector<vector<pair<int, double>>>& gr) {
    vector<int> dist(n, 1e9), parent(n, -1);
    queue<int> q;

    dist[s] = 0;
    q.push(s);

    while (!q.empty()) {
        int cur = q.front();
        q.pop();

        for (int i = 0; i < gr[cur].size(); i++) {
            int next = gr[cur][i].first;
            if (dist[cur] + 1 < dist[next]) {
                dist[next] = dist[cur] + 1;
                parent[next] = cur;
                q.push(next);
            }
        }
    }

    if (dist[f] == 1e9) {
        return {}; // Пути нет
    }

    // Восстановление пути
    vector<int> path;
    for (int v = f; v != -1; v = parent[v]) {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

// DFS для поиска кратчайшего пути по числу узлов
bool dfsShortestPath(int node, int target, vector<vector<pair<int, double>>>& gr, vector<bool>& globalVisited, vector<bool>& visitedInPath, vector<int>& currentPath, vector<int>& shortestPath) {
    if (node == target) {
        if (shortestPath.empty() || currentPath.size() < shortestPath.size()) {
            shortestPath = currentPath;
        }
        return true;
    }

    globalVisited[node] = true;
    visitedInPath[node] = true;

    for (int i = 0; i < gr[node].size(); i++) {
        int next = gr[node][i].first;

        // Если вершина уже посещена на этом пути, пропускаем её
        if (!visitedInPath[next]) {
            currentPath.push_back(next);
            dfsShortestPath(next, target, gr, globalVisited, visitedInPath, currentPath, shortestPath);
            currentPath.pop_back(); // Backtrack
        }
    }

    
    return false;
}

// Dijkstra: Поиск кратчайшего пути с весами
vector<int> dijkstra(int n, int s, int f, vector<vector<pair<int, double>>>& gr) {
    vector<double> dist(n, 1e9);
    vector<int> parent(n, -1);
    dist[s] = 0;

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    pq.push(make_pair(0, s));

    while (!pq.empty()) {
        pair<double, int> cur = pq.top();
        pq.pop();

        if (cur.first > dist[cur.second]) continue;

        for (int i = 0; i < gr[cur.second].size(); i++) {
            int next = gr[cur.second][i].first;
            double weight = gr[cur.second][i].second;
            if (cur.first + weight < dist[next]) {
                dist[next] = cur.first + weight;
                parent[next] = cur.second;
                pq.push(make_pair(dist[next], next));
            }
        }
    }

    if (dist[f] == 1e9) {
        return {}; // Пути нет
    }

    // Восстановление пути
    vector<int> path;
    for (int v = f; v != -1; v = parent[v]) {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

#include <chrono>

using namespace std::chrono;

int main() {
    int n = 5;
    vector<Edge> edges = {
        {0, 1, 2, 2.0, 3.0},
        {1, 3, 4, 1.5, 2.5},
        {2, -1, 3, -1.0, 2.0},
        {3, 4, -1, 1.0, -1.0},
    };

    edges = readGraph("output.txt");
    n = 350881; //edges.size();

    vector<vector<pair<int, double>>> gr = buildGraph(n, edges);

    makeGraphUndirected(gr);

    int start = 37095, finish = 239852;

    cout << "Data saved" << endl;

    // BFS

    auto clock_start = high_resolution_clock::now();

    vector<int> bfsPath = bfs(n, start, finish, gr);

    auto clock_stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(clock_start - clock_stop);

    cout << "BFS Time: " << duration.count() / 1000 << endl;

    cout << "BFS Path:" << endl;

    if (bfsPath.empty()) {
        cout << "No path" << endl;
    }
    else {
        for (int i = 0; i < bfsPath.size(); i++) {
            cout << bfsPath[i] << " ";
        }
        cout << endl;
    }
    // DFS

    clock_start = high_resolution_clock::now();

    vector<bool> globalVisited(n, false);
    vector<bool> visitedInPath(n, false);
    vector<int> currentPath = { start };
    vector<int> shortestPath;

    //dfsShortestPath(start, finish, gr, globalVisited, visitedInPath, currentPath, shortestPath);

    clock_stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(clock_start - clock_stop);

    cout << "DFS Time: " << duration.count() / 1000 << endl;

    cout << "DFS Path: ";
    for (int i = 0; i < shortestPath.size(); i++) {
        cout << shortestPath[i] << " ";
    }
    cout << endl;


    


    // Dijkstra

    clock_start = high_resolution_clock::now();
    
    vector<int> dijkstraPath = dijkstra(n, start, finish, gr);

    clock_stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(clock_start - clock_stop);

    cout << "Dijkstra Time: " << duration.count() / 1000 << endl;

    cout << "Dijkstra Path:" << endl;

    if (dijkstraPath.empty()) {
        cout << "No path" << endl;
    }
    else {
        for (int i = 0; i < dijkstraPath.size(); i++) {
            cout << dijkstraPath[i] << " ";
        }
        cout << endl;
    }

    return 0;
}