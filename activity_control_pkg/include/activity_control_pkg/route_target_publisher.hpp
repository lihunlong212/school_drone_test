#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

struct Target
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
};

struct PillarTaskResult
{
  std::size_t pillar_index;
  double x_m;
  double y_m;
  double pillar_height_cm;
  int circle_rank;
  bool valid;
};

enum class MissionPhase : std::uint8_t
{
  WAIT_PILLARS = 0,
  TAKEOFF_TO_CRUISE = 1,
  VISIT_PILLAR = 2,
  HOVER_AND_MEASURE = 3,
  GO_TO_LAND_POINT = 4,
  LAND = 5,
  COMPLETE = 6,
};

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publishTarget(const Target & target);
  void publishActiveController(std::uint8_t controller_id);
  void publishVisualTakeoverState(bool active);
  void publishPillarTaskResult(const PillarTaskResult & result);
  void logMissionSummary() const;

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;
  double normalizeAngleDeg(double angle_deg) const;

  void pillarPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void circleRankCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void monitorTimerCallback();

  void startHoverMeasurement();
  void sampleHoverMeasurement();
  void finishCurrentPillarMeasurement();
  void completeMission(const std::string & reason);

  static double meterToCm(double value_m);
  static double cmToMeter(double value_cm);
  static double radToDeg(double value_rad);

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_takeover_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pillar_task_result_pub_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr circle_rank_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pillar_position_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;

  std::string map_frame_;
  std::string laser_link_frame_;
  std::string output_topic_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;

  double start_x_cm_;
  double start_y_cm_;
  double cruise_height_cm_;
  double landing_x_cm_;
  double landing_y_cm_;
  double mission_yaw_deg_;
  double hover_time_sec_;
  double pillar_height_threshold_cm_;

  bool has_height_;
  double current_height_cm_;
  int latest_circle_rank_;

  MissionPhase phase_;
  bool pillars_received_;
  bool mission_complete_sent_;
  bool has_current_target_;
  Target current_target_;
  std::size_t current_pillar_index_;

  std::vector<Target> pillar_targets_;
  std::vector<PillarTaskResult> pillar_results_;

  rclcpp::Time hover_start_time_;
  double hover_height_sum_cm_;
  std::size_t hover_height_sample_count_;
  std::map<int, int> hover_circle_rank_counts_;
};

}  // namespace activity_control_pkg
