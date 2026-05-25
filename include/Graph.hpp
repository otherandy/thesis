#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <optional>
#include <queue>
#include <stack>
#include <tuple>
#include <vector>

using Graph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS>;

using vertex_t = boost::graph_traits<Graph>::vertex_descriptor;
using edge_t = boost::graph_traits<Graph>::edge_descriptor;

class StepTraversal {
protected:
  Graph &g;

public:
  StepTraversal(Graph &g_, vertex_t start) : g(g_) {}
  virtual ~StepTraversal() = default;
  virtual std::optional<vertex_t> next() = 0;

  vertex_t add_vertex() { return boost::add_vertex(g); }

  edge_t add_edge(vertex_t u, vertex_t v) {
    return boost::add_edge(u, v, g).first;
  }

  vertex_t add_vertex_and_edge(vertex_t u) {
    vertex_t v = add_vertex();
    add_edge(u, v);
    return v;
  }

  virtual void post_update() {}
};

class StepDFS : public StepTraversal {
  std::vector<char> color; // white=0, gray=1, black=2
  std::stack<vertex_t> st;
  std::vector<typename boost::graph_traits<Graph>::out_edge_iterator> out_it;
  std::vector<typename boost::graph_traits<Graph>::out_edge_iterator> out_end;

public:
  StepDFS(Graph &g_, vertex_t start);

  std::optional<vertex_t> next() override;

  void post_update() override {
    const size_t n = boost::num_vertices(g);
    color.resize(n, 0);
    out_it.resize(n);
    out_end.resize(n);

    auto idx = boost::get(boost::vertex_index, g);
    auto [vi, vi_end] = boost::vertices(g);
    for (; vi != vi_end; ++vi) {
      const vertex_t v = *vi;
      std::tie(out_it[idx[v]], out_end[idx[v]]) = boost::out_edges(v, g);
    }
  }
};

class StepBFS : public StepTraversal {
  std::vector<char> color; // 0=white, 1=discovered, 3=active, 2=finished
  std::queue<vertex_t> q;
  std::vector<typename boost::graph_traits<Graph>::out_edge_iterator> out_it;
  std::vector<typename boost::graph_traits<Graph>::out_edge_iterator> out_end;

public:
  StepBFS(Graph &g_, vertex_t start);

  std::optional<vertex_t> next() override;

  void post_update() override {
    const size_t n = boost::num_vertices(g);
    color.resize(n, 0);
    out_it.resize(n);
    out_end.resize(n);

    auto idx = boost::get(boost::vertex_index, g);
    auto [vi, vi_end] = boost::vertices(g);
    for (; vi != vi_end; ++vi) {
      const vertex_t v = *vi;
      std::tie(out_it[idx[v]], out_end[idx[v]]) = boost::out_edges(v, g);
    }
  }
};

#endif
