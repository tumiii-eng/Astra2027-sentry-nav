#include "astra_perception/rolling_occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

namespace astra_nav
{

RollingOccupancyGrid3D::RollingOccupancyGrid3D(const OccupancyConfig & config)
: config_(config)
{
  nx_ = static_cast<int>(std::ceil(config_.size_x / config_.resolution));
  ny_ = static_cast<int>(std::ceil(config_.size_y / config_.resolution));
  nz_ = static_cast<int>(std::ceil(config_.size_z / config_.resolution));
  origin_ = {-0.5 * nx_ * config_.resolution, -0.5 * ny_ * config_.resolution, config_.z_min};
  log_odds_.assign(static_cast<std::size_t>(nx_ * ny_ * nz_), 0.0F);
  last_seen_.assign(log_odds_.size(), 0);
}

void RollingOccupancyGrid3D::update(
  const std::vector<Point3D> & points_world, const Point3D & sensor_origin_world,
  const Point3D & robot_center_world)
{
  ++tick_;
  shift_to_center(robot_center_world);

  auto start = world_to_index(sensor_origin_world);
  if (!start) {
    start = world_to_index(robot_center_world);
  }
  if (!start) {
    fade();
    return;
  }

  const std::size_t step =
    points_world.size() > config_.max_points_per_update && config_.max_points_per_update > 0
    ? std::max<std::size_t>(1, points_world.size() / config_.max_points_per_update)
    : 1;
  std::size_t used = 0;
  for (std::size_t i = 0; i < points_world.size(); i += step) {
    if (config_.max_points_per_update > 0 && used >= config_.max_points_per_update) {
      break;
    }
    const auto end = world_to_index(points_world[i]);
    if (!end) {
      continue;
    }
    raycast_update(*start, *end);
    ++used;
  }

  fade();
}

std::vector<bool> RollingOccupancyGrid3D::occupied_mask() const
{
  std::vector<bool> mask(log_odds_.size(), false);
  for (std::size_t i = 0; i < log_odds_.size(); ++i) {
    mask[i] = probability_from_log_odds(log_odds_[i]) >= config_.occupied_threshold;
  }
  return mask;
}

std::vector<Point3D> RollingOccupancyGrid3D::occupied_points() const
{
  std::vector<Point3D> points;
  for (int ix = 0; ix < nx_; ++ix) {
    for (int iy = 0; iy < ny_; ++iy) {
      for (int iz = 0; iz < nz_; ++iz) {
        if (occupied_at(ix, iy, iz)) {
          points.push_back(index_to_world(ix, iy, iz));
        }
      }
    }
  }
  return points;
}

std::optional<std::array<int, 3>> RollingOccupancyGrid3D::world_to_index(const Point3D & point) const
{
  const int ix = static_cast<int>(std::floor((point.x - origin_.x) / config_.resolution));
  const int iy = static_cast<int>(std::floor((point.y - origin_.y) / config_.resolution));
  const int iz = static_cast<int>(std::floor((point.z - origin_.z) / config_.resolution));
  if (ix < 0 || iy < 0 || iz < 0 || ix >= nx_ || iy >= ny_ || iz >= nz_) {
    return std::nullopt;
  }
  return std::array<int, 3>{ix, iy, iz};
}

Point3D RollingOccupancyGrid3D::index_to_world(int ix, int iy, int iz) const
{
  return {
    origin_.x + (static_cast<double>(ix) + 0.5) * config_.resolution,
    origin_.y + (static_cast<double>(iy) + 0.5) * config_.resolution,
    origin_.z + (static_cast<double>(iz) + 0.5) * config_.resolution};
}

std::size_t RollingOccupancyGrid3D::flat_index(int ix, int iy, int iz) const
{
  return static_cast<std::size_t>((ix * ny_ + iy) * nz_ + iz);
}

double RollingOccupancyGrid3D::probability_from_log_odds(double value) const
{
  return 1.0 - 1.0 / (1.0 + std::exp(value));
}

bool RollingOccupancyGrid3D::occupied_at(int ix, int iy, int iz) const
{
  return probability_from_log_odds(log_odds_[flat_index(ix, iy, iz)]) >= config_.occupied_threshold;
}

void RollingOccupancyGrid3D::shift_to_center(const Point3D & robot_center_world)
{
  const Point3D desired = {
    robot_center_world.x - 0.5 * nx_ * config_.resolution,
    robot_center_world.y - 0.5 * ny_ * config_.resolution,
    config_.z_min};
  const int dx = static_cast<int>(std::floor((desired.x - origin_.x) / config_.resolution));
  const int dy = static_cast<int>(std::floor((desired.y - origin_.y) / config_.resolution));
  if (std::abs(dx) < 1 && std::abs(dy) < 1) {
    return;
  }
  if (std::abs(dx) >= nx_ || std::abs(dy) >= ny_) {
    std::fill(log_odds_.begin(), log_odds_.end(), 0.0F);
    std::fill(last_seen_.begin(), last_seen_.end(), 0);
    origin_ = desired;
    return;
  }

  std::vector<float> next_log(log_odds_.size(), 0.0F);
  std::vector<int> next_seen(last_seen_.size(), 0);
  for (int ix = 0; ix < nx_; ++ix) {
    for (int iy = 0; iy < ny_; ++iy) {
      for (int iz = 0; iz < nz_; ++iz) {
        const int sx = ix + dx;
        const int sy = iy + dy;
        if (sx < 0 || sy < 0 || sx >= nx_ || sy >= ny_) {
          continue;
        }
        const auto dst = flat_index(ix, iy, iz);
        const auto src = flat_index(sx, sy, iz);
        next_log[dst] = log_odds_[src];
        next_seen[dst] = last_seen_[src];
      }
    }
  }
  log_odds_.swap(next_log);
  last_seen_.swap(next_seen);
  origin_.x += dx * config_.resolution;
  origin_.y += dy * config_.resolution;
}

void RollingOccupancyGrid3D::raycast_update(
  const std::array<int, 3> & start, const std::array<int, 3> & end)
{
  const int dx = end[0] - start[0];
  const int dy = end[1] - start[1];
  const int dz = end[2] - start[2];
  const int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz), 1});

  for (int s = 0; s <= steps; ++s) {
    const double ratio = static_cast<double>(s) / static_cast<double>(steps);
    const int ix = static_cast<int>(std::round(start[0] + ratio * dx));
    const int iy = static_cast<int>(std::round(start[1] + ratio * dy));
    const int iz = static_cast<int>(std::round(start[2] + ratio * dz));
    if (ix < 0 || iy < 0 || iz < 0 || ix >= nx_ || iy >= ny_ || iz >= nz_) {
      continue;
    }
    const auto idx = flat_index(ix, iy, iz);
    if (s == steps) {
      log_odds_[idx] = static_cast<float>(
        std::clamp<double>(log_odds_[idx] + config_.log_odds_hit, config_.log_odds_min, config_.log_odds_max));
      last_seen_[idx] = tick_;
    } else {
      log_odds_[idx] = static_cast<float>(
        std::clamp<double>(log_odds_[idx] + config_.log_odds_miss, config_.log_odds_min, config_.log_odds_max));
    }
  }
}

void RollingOccupancyGrid3D::fade()
{
  if (config_.fade_ticks <= 0) {
    return;
  }
  for (std::size_t i = 0; i < log_odds_.size(); ++i) {
    if (
      probability_from_log_odds(log_odds_[i]) >= config_.occupied_threshold &&
      tick_ - last_seen_[i] > config_.fade_ticks)
    {
      log_odds_[i] = static_cast<float>(std::max(0.0, log_odds_[i] + 2.0 * config_.log_odds_miss));
    }
  }
}

}  // namespace astra_nav

