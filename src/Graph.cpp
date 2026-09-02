#include "Graph.hpp"
#include <algorithm>
#include <optional>

vertex_t DynamicScheduler::add_vertex(std::shared_ptr<FrontierRegion> region,
                                      const Grid2D<std::unique_ptr<Cell>> &grid,
                                      bool root) {
  std::lock_guard<std::mutex> lg(mutex_);

  const double area = region->get_area_slow(grid);

  vertex_t v = boost::add_vertex(
      VertexData{next_id_++, std::move(region), VertexData::Color::White, area},
      g_);

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
    dfs_stack_.emplace_back(v);
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

std::optional<vertex_t> DynamicScheduler::next() {
  std::optional<vertex_t> vopt;

  auto results(dfs_stack_);

  auto compare = [&](vertex_t v1, vertex_t v2) {
    auto v1d = get_vertex_data(v1);
    auto v2d = get_vertex_data(v2);

    return v1d.area > v2d.area;
  };

  if (!results.empty()) {
    std::sort(results.begin(), results.end(), compare);
    vopt = results.front();
  }

  if (!vopt.has_value()) {
    return std::nullopt;
  }

  dfs_stack_.erase(std::remove_if(dfs_stack_.begin(), dfs_stack_.end(),
                                  [&](vertex_t v) { return v == *vopt; }),
                   dfs_stack_.end());

  vertex_t v = *vopt;

  g_[v].workers++;

  return v;
}

std::optional<vertex_t> DynamicScheduler::help() {
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

std::optional<vertex_t> DynamicScheduler::next_or_help() {
  auto vopt = next();

  if (!vopt.has_value()) {
    vopt = help();
  }

  return vopt;
}

std::optional<vertex_t>
DynamicScheduler::closest(const Grid2D<std::unique_ptr<Cell>> &grid,
                          const Robot::Point &position) {
  auto vertices = get_all_vertices();

  if (vertices.empty()) {
    return std::nullopt;
  }

  double closest_distance = std::numeric_limits<double>::max();
  std::optional<vertex_t> closest_v;

  for (auto v : vertices) {
    if (is_done(v)) {
      continue;
    }

    auto vd = get_vertex_data(v);
    const auto c = vd.region->get_closest_unexplored(grid, position);

    if (c.has_value()) {
      const double d = CGAL::squared_distance(position, c.value());

      if (d < closest_distance) {
        closest_distance = d;
        closest_v = v;
      }
    }
  }

  if (!closest_v.has_value()) {
    return std::nullopt;
  }

  g_[*closest_v].workers++;
  return closest_v;
}

std::optional<vertex_t>
DynamicScheduler::largest_approx(const Grid2D<std::unique_ptr<Cell>> &grid) {
  auto vertices = get_all_vertices();

  if (vertices.empty()) {
    return std::nullopt;
  }

  size_t largest_area = 0;
  std::optional<vertex_t> biggest_v;

  for (auto v : vertices) {
    if (is_done(v)) {
      continue;
    }

    auto vd = get_vertex_data(v);
    const double area = vd.region->get_area();

    if (area > largest_area) {
      largest_area = area;
      biggest_v = v;
    }
  }

  if (!biggest_v.has_value()) {
    return std::nullopt;
  }

  g_[*biggest_v].workers++;
  return biggest_v;
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

    DrawText(TextFormat("%.2f", data.area), (int)(p.x + 4), (int)(p.y + 1), 10,
             BLACK);
  }
}
