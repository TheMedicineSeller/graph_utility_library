#pragma once

#include <vector>
#include <queue>
/* ----------------GRAPH ALGORITHMS & USEFUL FUNCTIONS------------------
    All edges are assumed to be positively weighted
*/


// A depth first search function for both adjacency list and matrix implementations of graph using vector<>. Visited passed as reference can be used for analysis
void DFS (std::vector<std::vector<int>> G, int current, std::vector<bool> &visited) {
    visited[current] = true;
    for (int neighbour = 0; neighbour < G[current].size(); neighbour ++)
        if (!visited[neighbour] && (G[current][neighbour] > 0))
            DFS(G, neighbour, visited);
}
// A breadth first search function for both adjacency list and matrix implementations of graph using vector<>.Visited passed as reference can be used for analysis
void BFS (std::vector<std::vector<int>> G, int start, std::vector<bool> &visited) {
    std::queue<int> Q;
    Q.push(start);
    visited[start] = true;
    while(!Q.empty()){
        for (int neighbour = 0; neighbour < G[start].size(); neighbour ++)
            if (!visited[neighbour] && (G[start][neighbour] > 0)) {
                Q.push(neighbour);
                visited[neighbour] = true;
            }
        Q.pop();
    }
}
// Returns the total no of connected components of a graph given by an Adjacency matrix
int Components (std::vector<std::vector<int>> G) {
    std::vector<bool> visited;
    visited.reserve(G.size());
    for (int i = 0; i < G.size(); i ++)
        visited.emplace_back(false);
    int dfs_count = 0, untraversed = 0;
    while (true) {
        DFS(G, untraversed, visited);
        dfs_count ++;
        auto ptr = std::find(visited.begin(), visited.end(), false);
        if (ptr == visited.end())
            return dfs_count;
        untraversed = ptr - visited.begin();
    }
}

// Returns a list of all paths from a given source to destination of a graph (both adjacency matrix & list) that have a path-weight greater than or equal to k
std::vector<std::vector<int>> Kpaths (std::vector<std::vector<int>> G, int source, int destination, int k) {
    std::vector<int> path;
    std::vector<bool> visited;
    std::vector<std::vector<int>> kpaths;
    visited.reserve(G.size());
    for (int i = 0; i < G.size(); i ++)
        visited.emplace_back(false);

    Kpaths_utility(G, source, destination, path, 0, visited, k, kpaths);
    return kpaths;
}
// Recursive utility function for the same
void Kpaths_utility (std::vector<std::vector<int>> G, int current, int destination, std::vector<int> path, int weight, std::vector<bool> visited, int k, std::vector<std::vector<int>> &outPaths) {
    if (!visited[current]) {
        visited[current] = true;
        if (path.size() > 0)
            weight += G[path.size() - 1][current];
        path.push_back(current);
        if (current == destination) {
            if (weight >= k)
                outPaths.push_back(path);
            visited[current] = false;
            path.pop_back();
            weight -= G[path[path.size() - 1]][destination];
            return;
        }
        for (int itr = 0; itr < G[current].size(); itr ++) {
            if ((G[current][itr] != 0) && (!visited[itr]))
                Kpaths_utility (G, itr, destination, path, weight, visited, k, outPaths);   
        }
        path.pop_back();
    }
}

// Dijsktra algorithm (BFS implementation), for both adjacency matrix and list representations, returns a vector of shortest distances from given source node to all other nodes
std::vector<int> Dijsktra (std::vector<std::vector<int>> G,  int source) {
    std::vector<int> distances;
    std::vector<bool> visited;
    std::queue<int> Q;
    
    int n = G.size();
    distances.reserve(n);
    visited.reserve(n);
    for (int i = 0; i < n; i ++) {
        distances.emplace_back(INT_MAX);
        visited.emplace_back(false);
    }
    Q.push(source);
    distances[source] = 0;
    
    while (!Q.empty()) {
        source = Q.front();
        visited[source] = true;
        for (int neighbour = 0; neighbour < G[source].size(); neighbour ++)
            if (G[source][neighbour] > 0){
                distances[neighbour] = std::min(distances[neighbour], distances[source] + G[source][neighbour]);
                if (!visited[neighbour])
                    Q.push(neighbour);
            }
        Q.pop();
    }
    return distances;
}

// Checks whether a given graph (as both Adjacency matrix or list) is Bipartite or not
bool isBipartite (std::vector<std::vector<int>> G) {
    enum colouring {
        none = -1,
        red,
        blue
    };
    std::vector<colouring> colours;
    colours.reserve(G.size());
    for (int i = 0; i < G.size(); i ++)
        colours.emplace_back(none);
    std::queue<int> Q;
    Q.push(0); 
    colours[0] = red;

    while (!Q.empty()) {
        int vert = Q.front();
        Q.pop();
        for (int neighbour = 0; neighbour < G[vert].size(); neighbour ++) {
            if ((G[vert][neighbour] > 0) && (colours[neighbour] == none)) {
                colours[neighbour] = colouring(1 - colours[vert]);
                Q.push(neighbour);
            }
            else if ((G[vert][neighbour] > 0) && (colours[neighbour] == colours[vert]))
                return false;
        }
    }
    return true;
}
