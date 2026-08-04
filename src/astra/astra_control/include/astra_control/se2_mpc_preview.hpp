#pragma once

#include <vector>

#include "astra_common/common.hpp"

namespace astra_nav
{

struct Se2MpcPreviewConfig
{
  int steps{12};
  double dt{0.05};
};

struct Se2MpcPreviewResult
{
  bool valid{false};
  int steps{0};
  double horizon_duration{0.0};
  double mean_position_error{0.0};
  double max_position_error{0.0};
  double final_position_error{0.0};
  TrajectoryPoint final_prediction;
  TrajectoryPoint final_reference;
};

class Se2MpcPreview
{
public:
  explicit Se2MpcPreview(const Se2MpcPreviewConfig & config);

  Se2MpcPreviewResult predict(
    const Pose2D & pose, const Twist2D & held_command,
    const std::vector<TrajectoryPoint> & trajectory, double current_time_from_start) const;

private:
  TrajectoryPoint nearest_reference(
    const std::vector<TrajectoryPoint> & trajectory, double target_time) const;

  Se2MpcPreviewConfig config_;
};

}  // namespace astra_nav
