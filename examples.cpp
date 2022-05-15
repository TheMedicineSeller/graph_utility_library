#include <iostream>
#include "graph.h"
using namespace std;

void main () {
  vector<vector<int>> G{
    {0, 0, 0, 1},
    {1, 0, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 1, 0}
  };
  if (isBipartite(G))
    cout<< "Given graph's vertices can be partitioned into two sets such that there exists no edges between the vertices of a set.";
  else
    cout<< "Nope";
  
  return 0;
}
