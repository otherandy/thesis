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
    bfs_queue_.push(v);
  }
}

void DynamicScheduler::mark_done(vertex_t v) {
  std::lock_guard<std::mutex> lg(mutex_);
  if (g_[v].color != VertexData::Color::Black) {
    g_[v].color = VertexData::Color::Black;
  }
}

bool DynamicScheduler::is_done(vertex_t v) {
  std::lock_guard<std::mutex> lg(mutex_);
  return g_[v].color == VertexData::Color::Black;
}

std::optional<vertex_t> DynamicScheduler::next(const std::string &strategy) {
  std::lock_guard<std::mutex> lg(mutex_);
  std::optional<vertex_t> vopt;

  if (strategy == "dfs") {
    vopt = pop_dfs();
  } else if (strategy == "bfs") {
    vopt = pop_bfs();
  }

  if (!vopt.has_value()) {
    return std::nullopt;
  }

  vertex_t v = *vopt;

  g_[v].workers++;

  return v;
}

std::optional<vertex_t> DynamicScheduler::help(const std::string &strategy) {
  std::lock_guard<std::mutex> lg(mutex_);

  auto vertices = get_all_vertices();
  vertices.erase(std::remove_if(vertices.begin(), vertices.end(),
                                [this](vertex_t v) {
                                  return g_[v].color != VertexData::Color::Gray;
                                }),
                 vertices.end());

  if (vertices.empty()) {
    return std::nullopt;
  }

  auto compare = [&](vertex_t a, vertex_t b) {
    return g_[a].workers < g_[b].workers;
  };

  std::sort(vertices.begin(), vertices.end(), compare);

  vertex_t v = vertices.front();
  g_[v].workers++;

  return v;
}

std::optional<vertex_t>
DynamicScheduler::next_or_help(const std::string &strategy) {
  auto vopt = next(strategy);

  if (!vopt.has_value()) {
    vopt = help(strategy);
  }

  return vopt;
}

std::optional<vertex_t>
DynamicScheduler::closest(const Grid2D<std::unique_ptr<Cell>> &grid,
                          const Robot::Point &position) {
  std::lock_guard<std::mutex> lg(mutex_);
  auto vertices = get_all_vertices();

  if (vertices.empty()) {
    return std::nullopt;
  }

  double closest_distance = std::numeric_limits<double>::max();
  vertex_t closest_v;

  for (auto v : vertices) {
    auto vd = g_[v];
    const auto c = vd.region->get_closest_unexplored(grid, position);

    if (c.has_value()) {
      const double d = CGAL::squared_distance(position, c.value());

      if (d < closest_distance) {
        closest_distance = d;
        closest_v = v;
      }
    }
  }

  g_[closest_v].workers++;
  return closest_v;
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

bool DynamicScheduler::finished() {
  for (auto v : get_all_vertices()) {
    if (g_[v].color == VertexData::Color::Gray) {
      return false;
    }
  }
  return true;
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
    positions_[i] = {cx + radius * std::cos(t) + screenW,
                     cy + radius * std::sin(t)};
  }

  layout_dirty_ = false;
}

static Color toRayColor(VertexData::Color c) {
  switch (c) {
  case VertexData::Color::White:
    return raylib::WHITE;
  case VertexData::Color::Gray:
    return raylib::YELLOW;
  case VertexData::Color::Black:
    return raylib::GRAY;
  }
  return raylib::RED;
}

void DynamicScheduler::draw(int screenW, int screenH) {
  ensure_layout(screenW, screenH);

  for (auto [ei, ei_end] = boost::edges(g_); ei != ei_end; ++ei) {
    const auto u = boost::source(*ei, g_);
    const auto v = boost::target(*ei, g_);

    if ((size_t)u >= positions_.size() || (size_t)v >= positions_.size()) {
      continue;
    }

    raylib::Vector2 p1 = positions_[u];
    raylib::Vector2 p2 = positions_[v];

    DrawLineV(p1, p2, Fade(BLUE, 0.6f));
  }

  for (auto [vi, vi_end] = boost::vertices(g_); vi != vi_end; ++vi) {
    const auto idx = (size_t)(*vi); // vecS => stable integer indices
    const raylib::Vector2 p = positions_[idx];

    const auto data = g_[*vi];
    const float r = 14.0f;

    DrawCircleV(p, r, toRayColor(data.color));
    DrawCircleLines((int)p.x, (int)p.y, r, BLACK);

    DrawText(TextFormat("%zu", data.id), (int)(p.x - 10), (int)(p.y - 7), 10,
             BLACK);

    DrawText(TextFormat("%zu", data.workers), (int)(p.x + 4), (int)(p.y + 1),
             10, BLACK);
  }
}
