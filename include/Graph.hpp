#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "FrontierRegion.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <queue>
#include <stack>

struct VertexData {
  std::size_t id;
  std::shared_ptr<FrontierRegion> region;

  enum class Color { White, Gray, Black } color = Color::White;

  std::size_t workers = 0;
};

using Graph = boost::adjacency_list<boost::vecS,        // OutEdgeList
                                    boost::vecS,        // VertexList
                                    boost::undirectedS, // UndirectedGraph
                                    VertexData>;

using vertex_t = boost::graph_traits<Graph>::vertex_descriptor;
using edge_t = boost::graph_traits<Graph>::edge_descriptor;

class DynamicScheduler {
public:
  DynamicScheduler() {}

  vertex_t add_vertex(std::shared_ptr<FrontierRegion> region,
                      bool root = false);
  void add_edge(vertex_t u, vertex_t v);

  std::optional<vertex_t> next(const std::string &strategy = "dfs");
  std::optional<vertex_t> help(const std::string &strategy = "dfs");
  std::optional<vertex_t> next_or_help(const std::string &strategy = "dfs");
  std::optional<vertex_t> closest(const Grid2D<std::unique_ptr<Cell>> &grid,
                                  const Robot::Point &position);
  void done(vertex_t v);
  bool is_done(vertex_t v);

  VertexData get_vertex_data(vertex_t v);
  std::vector<vertex_t> get_all_vertices();
  bool finished();

  void mark_layout_dirty() { layout_dirty_ = true; }
  void draw(int screenW, int screenH);
  void ensure_layout(int screenW, int screenH);

  Graph &graph() { return g_; }

private:
  std::vector<Vector2> positions_;
  bool layout_dirty_ = true;

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
