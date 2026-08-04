#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "astra_common/common.hpp"

namespace astra_nav
{

std::vector<Point3D> read_xyz_points(
  const sensor_msgs::msg::PointCloud2 & msg, std::size_t max_points);

sensor_msgs::msg::PointCloud2 make_xyz_cloud(
  const std::vector<Point3D> & points, const std::string & frame_id, const rclcpp::Time & stamp);

}  // namespace astra_nav

