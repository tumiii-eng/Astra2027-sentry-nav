#include "astra_perception/pointcloud_utils.hpp"

#include <cstring>
#include <unordered_map>

#include <sensor_msgs/msg/point_field.hpp>

namespace astra_nav
{

std::vector<Point3D> read_xyz_points(
  const sensor_msgs::msg::PointCloud2 & msg, std::size_t max_points)
{
  std::unordered_map<std::string, std::size_t> offsets;
  for (const auto & field : msg.fields) {
    offsets[field.name] = field.offset;
  }
  if (!offsets.count("x") || !offsets.count("y") || !offsets.count("z") || msg.point_step == 0) {
    return {};
  }

  const std::size_t count =
    static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
  const std::size_t step =
    max_points > 0 && count > max_points ? std::max<std::size_t>(1, count / max_points) : 1;
  std::vector<Point3D> points;
  points.reserve(std::min(count, max_points == 0 ? count : max_points));

  for (std::size_t i = 0; i < count; i += step) {
    const std::size_t base = i * msg.point_step;
    if (base + msg.point_step > msg.data.size()) {
      break;
    }
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::memcpy(&x, &msg.data[base + offsets["x"]], sizeof(float));
    std::memcpy(&y, &msg.data[base + offsets["y"]], sizeof(float));
    std::memcpy(&z, &msg.data[base + offsets["z"]], sizeof(float));
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      points.push_back({x, y, z});
    }
    if (max_points > 0 && points.size() >= max_points) {
      break;
    }
  }
  return points;
}

sensor_msgs::msg::PointCloud2 make_xyz_cloud(
  const std::vector<Point3D> & points, const std::string & frame_id, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = 3 * sizeof(float);
  msg.row_step = msg.point_step * msg.width;
  msg.fields.resize(3);
  const char * names[3] = {"x", "y", "z"};
  for (std::size_t i = 0; i < 3; ++i) {
    msg.fields[i].name = names[i];
    msg.fields[i].offset = static_cast<std::uint32_t>(i * sizeof(float));
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }
  msg.data.resize(static_cast<std::size_t>(msg.row_step));
  for (std::size_t i = 0; i < points.size(); ++i) {
    const float values[3] = {
      static_cast<float>(points[i].x),
      static_cast<float>(points[i].y),
      static_cast<float>(points[i].z)};
    std::memcpy(&msg.data[i * msg.point_step], values, sizeof(values));
  }
  return msg;
}

}  // namespace astra_nav

