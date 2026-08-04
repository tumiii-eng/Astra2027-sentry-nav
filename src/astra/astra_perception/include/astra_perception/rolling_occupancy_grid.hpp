#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "astra_common/common.hpp"

namespace astra_nav
{

struct OccupancyConfig
{
  double resolution{0.1};
  double size_x{10.0};
  double size_y{10.0};
  double size_z{1.2};
  double z_min{-0.15};
  double log_odds_hit{0.85};
  double log_odds_miss{-0.35};
  double log_odds_min{-3.5};
  double log_odds_max{4.0};
  double occupied_threshold{0.6};
  int fade_ticks{35};
  std::size_t max_points_per_update{2500};
};

class RollingOccupancyGrid3D
{
public:
  explicit RollingOccupancyGrid3D(const OccupancyConfig & config);

  void update(
    const std::vector<Point3D> & points_world, const Point3D & sensor_origin_world,
    const Point3D & robot_center_world);

  std::vector<bool> occupied_mask() const;
  std::vector<Point3D> occupied_points() const;
  std::optional<std::array<int, 3>> world_to_index(const Point3D & point) const;
  Point3D index_to_world(int ix, int iy, int iz) const;

  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }
  double resolution() const { return config_.resolution; }
  Point3D origin() const { return origin_; }
  const OccupancyConfig & config() const { return config_; }

private:
  std::size_t flat_index(int ix, int iy, int iz) const;
  double probability_from_log_odds(double value) const;
  bool occupied_at(int ix, int iy, int iz) const;
  void shift_to_center(const Point3D & robot_center_world);
  void clear_shifted_regions(int dx, int dy);
  void raycast_update(const std::array<int, 3> & start, const std::array<int, 3> & end);
  void fade();

  OccupancyConfig config_;
  int nx_{0};
  int ny_{0};
  int nz_{0};
  Point3D origin_;
  int tick_{0};
  std::vector<float> log_odds_;
  std::vector<int> last_seen_;
};

}  // namespace astra_nav

