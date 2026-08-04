#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace astra_nav
{

class BatchLiwoProxyNode : public rclcpp::Node
{
public:
  BatchLiwoProxyNode()
  : Node("batch_liwo_proxy_node")
  {
    input_topic_ = declare_parameter<std::string>("input_odom_topic", "/sim/odom");
    output_topic_ = declare_parameter<std::string>("output_odom_topic", "/astra/odom");
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 20);
    subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, 50, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        publisher_->publish(*msg);
      });
    RCLCPP_WARN(
      get_logger(),
      "当前使用 Batch-LIWO 接口代理，仅用于第一阶段仿真闭环；后续必须替换为完整 C++ 里程计。");
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
};

}  // namespace astra_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astra_nav::BatchLiwoProxyNode>());
  rclcpp::shutdown();
  return 0;
}

