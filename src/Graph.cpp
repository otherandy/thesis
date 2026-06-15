#include "Graph.hpp"

vertex_t DynamicScheduler::add_vertex(std::shared_ptr<FrontierRegion> region) {
  std::lock_guard<std::mutex> lg(mutex_);
  vertex_t v = boost::add_vertex(VertexData{next_id_++, region}, g_);
  return v;
}

void DynamicScheduler::add_edge(vertex_t u, vertex_t v) {
  std::lock_guard<std::mutex> lg(mutex_);
  boost::add_edge(u, v, g_);
}

void DynamicScheduler::start_from(vertex_t root, bool use_bfs) {
  std::lock_guard<std::mutex> lg(mutex_);
  if (use_bfs) {
    if (g_[root].color == VertexData::Color::White) {
      bfs_queue_.push(root);
      g_[root].color = VertexData::Color::Gray;
      g_[root].discover_time = ++time_;
    }
  } else {
    if (g_[root].color == VertexData::Color::White) {
      dfs_stack_.push(root);
      g_[root].color = VertexData::Color::Gray;
      g_[root].discover_time = ++time_;
    }
  }
}

std::vector<vertex_t>
DynamicScheduler::next_nodes(std::size_t k, const std::string &strategy) {
  std::lock_guard<std::mutex> lg(mutex_);
  std::vector<vertex_t> out;
  while (out.size() < k) {
    std::optional<vertex_t> vopt;
    if (strategy == "bfs")
      vopt = pop_bfs();
    else
      vopt = pop_dfs();

    if (!vopt)
      break;
    vertex_t v = *vopt;
    // Mark vertex as being processed by caller (we keep it Gray until caller
    // reports done)
    out.push_back(v);

    // Pre-fill neighbors as new frontier (for dynamic work distribution)
    for (auto ei = boost::adjacent_vertices(v, g_); ei.first != ei.second;
         ++ei.first) {
      vertex_t n = *ei.first;
      if (g_[n].color == VertexData::Color::White) {
        g_[n].color = VertexData::Color::Gray;
        g_[n].discover_time = ++time_;
        if (strategy == "bfs")
          bfs_queue_.push(n);
        else
          dfs_stack_.push(n);
      }
    }

    // Immediately mark v finished (this design assumes tasks are quick;
    // otherwise provide a separate done(v) call so workers mark finish when
    // truly done)
    g_[v].finish_time = ++time_;
    g_[v].color = VertexData::Color::Black;
  }
  return out;
}

void DynamicScheduler::done(vertex_t v) {
  std::lock_guard<std::mutex> lg(mutex_);
  if (g_[v].color != VertexData::Color::Black) {
    g_[v].finish_time = ++time_;
    g_[v].color = VertexData::Color::Black;
  }
}

VertexData DynamicScheduler::get_vertex_data(vertex_t v) {
  std::lock_guard<std::mutex> lg(mutex_);
  return g_[v];
}

std::vector<vertex_t> DynamicScheduler::all_vertices() {
  std::vector<vertex_t> out;
  for (auto vp = vertices(g_); vp.first != vp.second; ++vp.first)
    out.push_back(*vp.first);
  return out;
}

std::optional<vertex_t> DynamicScheduler::pop_dfs() {
  while (!dfs_stack_.empty()) {
    vertex_t v = dfs_stack_.top();
    dfs_stack_.pop();
    if (g_[v].color == VertexData::Color::Gray) {
      // we return it for processing
      return v;
    }
    // skip if already Black
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
