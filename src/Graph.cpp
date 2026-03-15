#include "Graph.hpp"

incremental_dfs::incremental_dfs(const Graph &g, Vertex start)
    : g_(g)
{
  stack_.push(start);
  visited_.insert(start);
}

bool incremental_dfs::next(Vertex &out)
{
  if (stack_.empty())
  {
    return false;
  }

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