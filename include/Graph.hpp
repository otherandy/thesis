#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <stack>

typedef boost::adjacency_list<
    boost::vecS, boost::vecS,
    boost::undirectedS>
    Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

class incremental_dfs
{
private:
  const Graph &g_;
  std::stack<Vertex> stack_;
  std::unordered_set<Vertex> visited_;

public:
  incremental_dfs(const Graph &g, Vertex start);

  bool next(Vertex &out);
};

#endif