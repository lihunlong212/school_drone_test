#include <clocale>

#include "activity_control_pkg/route_target_publisher.hpp"

int main(int argc, char ** argv)
{
  std::setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);

  auto node = std::make_shared<activity_control_pkg::RouteTargetPublisherNode>();
  RCLCPP_WARN(
    node->get_logger(),
    "route_test_node now runs the dynamic pillar-driven mission flow. Static route test mode is disabled.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
