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
  incremental_dfs(const Graph &g, Vertex start)
      : g_(g)
  {
    stack_.push(start);
    visited_.insert(start);
  }

  bool next(Vertex &out)
  {
    if (stack_.empty())
      return false;

    Vertex u = stack_.top();
    stack_.pop();
    out = u;

    auto [ei, ei_end] = boost::out_edges(u, g_);
    for (; ei != ei_end; ++ei)
    {
      Vertex v = boost::target(*ei, g_);
      if (!visited_.count(v))
      {
        visited_.insert(v);
        stack_.push(v);
      }
    }
    return true;
  }
};

#endif