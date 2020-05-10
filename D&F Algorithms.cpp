/*
**************************************************************************************************************************
* Name: Mohammad Tayseer Mohammad Abu Mailiesh
* Date: May, 8th 2020
* Project: Dijkstra and Floyd Marshell Algorithms
* Credits: geekforgeeks
* Overview: A program that finds the shortest path by importing
*           data from a .txt file into an array then printing the
*           path using two different algorithms
**************************************************************************************************************************
*/

#include <stdio.h> 
#include <limits.h> 
#include <iostream>
#include <fstream>
#include <climits>
#include <iomanip>

using namespace std;

//Number of vertices in the graph
//Change to suit your text file
#define V 9 

int finalnode;
int src;

// Recursive Function to print path of given 
// vertex u from source vertex v
void printPath1(int path[][V], int v, int u)
{
    if (path[v][u] == v)
        return;

    printPath1(path, v, path[v][u]);
    cout << path[v][u] << " ";
}

// Function to print the shortest cost with path 
// information between all pairs of vertices
void printSolution1(int cost[V][V], int path[V][V])
{
    cout << endl;
    for (int v = 0; v < V; v++)
    {
        for (int u = 0; u < V; u++)
        {
            if (u != v && path[v][u] != -1)
            {
                if (src == v && finalnode == u) {
                    cout << "Shortest Path from vertex " << v <<
                        " to vertex " << u << " is (" << v << " ";
                    printPath1(path, v, u);
                    cout << u << ")" << endl;
                }
            }
        }
    }
}

// Function to run Floyd-Warshell algorithm
void FloydWarshell(int adjMatrix[][V])
{
    // cost[] and parent[] stores shortest-path 
    // (shortest-cost/shortest route) information
    int cost[V][V], path[V][V];

    // initialize cost[] and parent[]
    for (int v = 0; v < V; v++)
    {
        for (int u = 0; u < V; u++)
        {
            // initally cost would be same as weight 
            // of the edge
            cost[v][u] = adjMatrix[v][u];

            if (v == u)
                path[v][u] = 0;
            else if (cost[v][u] != INT_MAX)
                path[v][u] = v;
            else
                path[v][u] = -1;
        }
    }

    // run Floyd-Warshell
    for (int k = 0; k < V; k++)
    {
        for (int v = 0; v < V; v++)
        {
            for (int u = 0; u < V; u++)
            {
                // If vertex k is on the shortest path from v to u,
                // then update the value of cost[v][u], path[v][u]

                if (cost[v][k] != INT_MAX && cost[k][u] != INT_MAX
                    && cost[v][k] + cost[k][u] < cost[v][u])
                {
                    cost[v][u] = cost[v][k] + cost[k][u];
                    path[v][u] = path[k][u];
                }
            }

            // if diagonal elements become negative, the
            // graph contains a negative weight cycle
            if (cost[v][v] < 0)
            {
                cout << "Negative Weight Cycle Found!!";
                return;
            }
        }
    }

    // Print the shortest path between all pairs of vertices
    printSolution1(cost, path);
}

int minDistance(int dist[],
    bool sptSet[])
{

    // Initialize min value 
    int min = INT_MAX, min_index;

    for (int v = 0; v < V; v++)
        if (sptSet[v] == false &&
            dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}

// Function to print shortest 
// path from source to j 
// using parent array 
void printPath(int parent[], int j)
{

    // Base Case : If j is source 
    if (parent[j] == -1)
        return;

    printPath(parent, parent[j]);

    printf("%d ", j);
}

// A utility function to print  
// the constructed distance 
// array 
void printSolution(int dist[], int n,
    int parent[], int g)
{

    printf("Vertex\t Distance\tPath");
    for (int i = 0; i < V; i++)
    {
        if (i == g) {
            printf("\n%d -> %d \t\t %d\t\t%d ",
                src, i, dist[i], src);
            printPath(parent, i);
        }
    }
}

// Funtion that implements Dijkstra's 
// single source shortest path 
// algorithm for a graph represented 
// using adjacency matrix representation 
void dijkstra(int graph[V][V], int src)
{

    // The output array. dist[i] 
    // will hold the shortest 
    // distance from src to i 
    int dist[V];

    // sptSet[i] will true if vertex 
    // i is included / in shortest 
    // path tree or shortest distance  
    // from src to i is finalized 
    bool sptSet[V];

    // Parent array to store 
    // shortest path tree 
    int parent[V];

    // Initialize all distances as  
    // INFINITE and stpSet[] as false 
    for (int i = 0; i < V; i++)
    {
        parent[src] = -1;
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }

    // Distance of source vertex  
    // from itself is always 0 
    dist[src] = 0;

    // Find shortest path 
    // for all vertices 
    for (int count = 0; count < V - 1; count++)
    {
        // Pick the minimum distance 
        // vertex from the set of 
        // vertices not yet processed.  
        // u is always equal to src 
        // in first iteration. 
        int u = minDistance(dist, sptSet);

        // Mark the picked vertex  
        // as processed 
        sptSet[u] = true;

        // Update dist value of the  
        // adjacent vertices of the 
        // picked vertex. 
        for (int v = 0; v < V; v++)

            // Update dist[v] only if is 
            // not in sptSet, there is 
            // an edge from u to v, and  
            // total weight of path from 
            // src to v through u is smaller 
            // than current value of 
            // dist[v] 
            if (!sptSet[v] && graph[u][v] &&
                dist[u] + graph[u][v] < dist[v])
            {
                parent[v] = u;
                dist[v] = dist[u] + graph[u][v];
            }
    }
    // print the constructed 
    // distance array 
    printSolution(dist, V, parent, finalnode);
}

// Driver Code 
int main()
{
    int graph[V][V];

    ifstream file{ "i.txt" };
    if (!file.is_open()) return -1;

    for (int i{}; i != V; ++i) {
        for (int j{}; j != V; ++j) {
            file >> graph[i][j];
        }
    }
    cout << "Enter the source & final node: ";
    cin >> src >> finalnode;
    cout << endl << endl;

    cout << "Shortest path using Dijkstra algorithm is: " << endl;
    dijkstra(graph, src);

    cout << endl << endl;

    cout << "Shortest path using Floyd Marshell algorithm is: " << endl;
    FloydWarshell(graph);

    cout << endl << endl;

    return 0;
}
