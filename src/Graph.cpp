#include "Graph.hpp"

vertex_t DynamicScheduler::add_vertex(std::shared_ptr<FrontierRegion> region,
                                      bool root) {
  std::lock_guard<std::mutex> lg(mutex_);

  vertex_t v = boost::add_vertex(
      VertexData{next_id_++, region, VertexData::Color::Black}, g_);

  if (!root && empty()) {
    push(v);
  }

  return v;
}

void DynamicScheduler::add_edge(vertex_t u, vertex_t v) {
  std::lock_guard<std::mutex> lg(mutex_);
  boost::add_edge(u, v, g_);
}

void DynamicScheduler::done(vertex_t v) {
  std::lock_guard<std::mutex> lg(mutex_);
  if (g_[v].color != VertexData::Color::Black) {
    g_[v].color = VertexData::Color::Black;
  }
}

std::optional<vertex_t> DynamicScheduler::next(const std::string &strategy) {
  std::lock_guard<std::mutex> lg(mutex_);
  std::optional<vertex_t> vopt;

  if (strategy == "dfs") {
    vopt = pop_bfs();
  } else if (strategy == "bfs") {
    vopt = pop_dfs();
  }

  if (!vopt.has_value()) {
    return std::nullopt;
  }

  vertex_t v = *vopt;

  for (auto ei = boost::adjacent_vertices(v, g_); ei.first != ei.second;
       ++ei.first) {
    vertex_t n = *ei.first;
    if (g_[n].color == VertexData::Color::White) {
      push(n);
    }
  }

  return v;
}

void DynamicScheduler::push(vertex_t v) {
  g_[v].color = VertexData::Color::Gray;
  bfs_queue_.push(v);
  dfs_stack_.push(v);
}

bool DynamicScheduler::empty() {
  return dfs_stack_.empty() || bfs_queue_.empty();
}

VertexData DynamicScheduler::get_vertex_data(vertex_t v) {
  std::lock_guard<std::mutex> lg(mutex_);
  return g_[v];
}

std::vector<vertex_t> DynamicScheduler::get_all_vertices() {
  std::vector<vertex_t> out;
  for (auto vp = vertices(g_); vp.first != vp.second; ++vp.first) {
    out.push_back(*vp.first);
  }
  return out;
}

std::optional<vertex_t> DynamicScheduler::pop_dfs() {
  while (!dfs_stack_.empty()) {
    vertex_t v = dfs_stack_.top();
    dfs_stack_.pop();
    if (g_[v].color == VertexData::Color::Gray) {
      return v;
    }
  }
  return std::nullopt;
}

std::optional<vertex_t> DynamicScheduler::pop_bfs() {
  while (!bfs_queue_.empty()) {
    vertex_t v = bfs_queue_.front();
    bfs_queue_.pop();
    if (g_[v].color == VertexData::Color::Gray) {
      return v;
    }
  }
  return std::nullopt;
}
