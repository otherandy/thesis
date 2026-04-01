#include "Graph.hpp"

StepDFS::StepDFS(Graph &g_, vertex_t start)
    : StepTraversal(g_), color(num_vertices(g_), 0),
      out_it(num_vertices(g_)), out_end(num_vertices(g_))
{
  auto idx = get(boost::vertex_index, g);
  st.push(start);
  color[idx[start]] = 1;
  std::tie(out_it[idx[start]], out_end[idx[start]]) = out_edges(start, g);
}

std::optional<vertex_t> StepDFS::next()
{
  auto idx = get(boost::vertex_index, g);
  while (!st.empty())
  {
    vertex_t v = st.top();
    if (color[idx[v]] == 1)
    {
      color[idx[v]] = 3;
      return v;
    }

    if (color[idx[v]] == 3)
    {
      for (; out_it[idx[v]] != out_end[idx[v]]; ++out_it[idx[v]])
      {
        edge_t e = *out_it[idx[v]];
        vertex_t u = target(e, g);
        if (color[idx[u]] == 0)
        {
          color[idx[u]] = 1;
          std::tie(out_it[idx[u]], out_end[idx[u]]) = out_edges(u, g);
          st.push(u);
          ++out_it[idx[v]];
          return u;
        }
      }
      color[idx[v]] = 2;
      st.pop();
      continue;
    }

    st.pop();
  }
  return std::nullopt;
}