#include "Graph.hpp"

vertex_t DynamicScheduler::add_vertex(std::shared_ptr<FrontierRegion> region,
                                      bool root) {
  std::lock_guard<std::mutex> lg(mutex_);

  vertex_t v = boost::add_vertex(
      VertexData{next_id_++, std::move(region), VertexData::Color::White}, g_);
  layout_dirty_ = true;

  if (root) {
    g_[v].color = VertexData::Color::Black;
  }

  return v;
}

void DynamicScheduler::add_edge(vertex_t u, vertex_t v) {
  std::lock_guard<std::mutex> lg(mutex_);
  boost::add_edge(u, v, g_);

  if (g_[v].color == VertexData::Color::White) {
    g_[v].color = VertexData::Color::Gray;
    dfs_stack_.push(v);
    dfs_stack_gray_.push(v);
    bfs_queue_.push(v);
    bfs_queue_gray_.push(v);
  }
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

  return v;
}

std::optional<vertex_t> DynamicScheduler::help(const std::string &strategy) {
  std::lock_guard<std::mutex> lg(mutex_);
  std::optional<vertex_t> vopt;

  if (strategy == "dfs") {
    vopt = pop_bfs_gray();
  } else if (strategy == "bfs") {
    vopt = pop_dfs_gray();
  }

  if (!vopt.has_value()) {
    return std::nullopt;
  }

  return *vopt;
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

std::optional<vertex_t> DynamicScheduler::pop_dfs_gray() {
  while (!dfs_stack_gray_.empty()) {
    vertex_t v = dfs_stack_gray_.top();
    if (g_[v].color == VertexData::Color::Black) {
      dfs_stack_gray_.pop();
    }
    if (g_[v].color == VertexData::Color::Gray) {
      return v;
    }
  }
  return std::nullopt;
}

std::optional<vertex_t> DynamicScheduler::pop_bfs_gray() {
  while (!bfs_queue_gray_.empty()) {
    vertex_t v = bfs_queue_gray_.front();
    if (g_[v].color == VertexData::Color::Black) {
      bfs_queue_gray_.pop();
    }
    if (g_[v].color == VertexData::Color::Gray) {
      return v;
    }
  }
  return std::nullopt;
}

void DynamicScheduler::ensure_layout(int screenW, int screenH) {
  if (!layout_dirty_) {
    return;
  }

  const size_t n = boost::num_vertices(g_);
  positions_.assign(n, {0.0f, 0.0f});

  if (n == 0) {
    layout_dirty_ = false;
    return;
  }

  const float cx = screenW * 0.5f;
  const float cy = screenH * 0.5f;
  const float radius = 0.45f * std::min(screenW, screenH);

  for (size_t i = 0; i < n; ++i) {
    float t = (n == 1) ? 0.0f : (2.0f * PI * (float)i / (float)n);
    positions_[i] = {cx + radius * std::cos(t), cy + radius * std::sin(t)};
  }

  layout_dirty_ = false;
}

static Color toRayColor(VertexData::Color c) {
  switch (c) {
  case VertexData::Color::White:
    return raylib::WHITE;
  case VertexData::Color::Gray:
    return raylib::GRAY;
  case VertexData::Color::Black:
    return raylib::DARKGRAY;
  }
  return raylib::RED;
}

void DynamicScheduler::draw(const DrawData &draw_data) {
  ensure_layout(draw_data.screen_x, draw_data.screen_y);

  for (auto [ei, ei_end] = boost::edges(g_); ei != ei_end; ++ei) {
    const auto u = boost::source(*ei, g_);
    const auto v = boost::target(*ei, g_);

    if ((size_t)u >= positions_.size() || (size_t)v >= positions_.size()) {
      continue;
    }

    Vector2 p1 = positions_[u];
    Vector2 p2 = positions_[v];

    DrawLineV(p1, p2, Fade(BLUE, 0.6f));
  }

  for (auto [vi, vi_end] = boost::vertices(g_); vi != vi_end; ++vi) {
    const auto idx = (size_t)(*vi); // vecS => stable integer indices
    const Vector2 p = positions_[idx];

    const auto data = g_[*vi];
    const float r = 14.0f;

    DrawCircleV(p, r, toRayColor(data.color));
    DrawCircleLines((int)p.x, (int)p.y, r, BLACK);

    DrawText(TextFormat("%zu", data.id), (int)(p.x - 10), (int)(p.y - 7), 10,
             BLACK);
  }
}
