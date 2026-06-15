#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "FrontierRegion.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <stack>
#include <queue>

struct VertexData {
  std::size_t id;
  FrontierRegion region;

  enum class Color { White, Gray, Black } color = Color::White;
  std::size_t discover_time = 0;
  std::size_t finish_time = 0;
};

using Graph = boost::adjacency_list<boost::vecS,        // OutEdgeList
                                    boost::vecS,        // VertexList
                                    boost::undirectedS, // UndirectedGraph
                                    VertexData>;

using vertex_t = boost::graph_traits<Graph>::vertex_descriptor;
using edge_t = boost::graph_traits<Graph>::edge_descriptor;

class DynamicScheduler {
public:
  DynamicScheduler() : time_(0) {}

  vertex_t add_node(FrontierRegion &region);
  void add_edge(vertex_t u, vertex_t v);

  // begin or resume a search from a vertex (pushes it as a root for DFS/BFS)
  void start_from(vertex_t root, bool use_bfs = false);

  // Request next up to `k` nodes to process. Strategy: "dfs" or "bfs".
  // Returns descriptors in visit order and marks them Gray/Black accordingly.
  std::vector<vertex_t> next_nodes(std::size_t k = 1,
                                   const std::string &strategy = "dfs");

  // Alternative: worker can claim a node and later call done(v) to mark finish.
  void done(vertex_t v);

  // Inspect vertex data (thread-safe snapshot)
  VertexData get_vertex_data(vertex_t v);

  Graph &graph() { return g_; }

private:
  std::optional<vertex_t> pop_dfs();
  std::optional<vertex_t> pop_bfs();

  Graph g_;
  std::mutex mutex_;
  std::stack<vertex_t> dfs_stack_;
  std::queue<vertex_t> bfs_queue_;
  std::size_t time_;
  std::size_t next_id_ = 0;
};

#endif
