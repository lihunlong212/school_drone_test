#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
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

struct PillarTaskSummarySlot
{
  std::size_t pillar_index;
  double x_m;
  double y_m;
  double pillar_height_cm;
  int circle_rank;
  std::uint8_t status;
};

enum class MissionPhase : std::uint8_t
{
  WAIT_PILLARS = 0,
  TAKEOFF_TO_CRUISE = 1,
  VISIT_PILLAR = 2,
  HOVER_AND_MEASURE = 3,
  GO_TO_LAND_POINT = 4,
  LAND_ALIGN = 5,
  LAND = 6,
  COMPLETE = 7,
  DELIVERY_GOTO_PICK = 8,
  DELIVERY_DESCEND_PICK = 9,
  DELIVERY_PICK_HOLD = 10,
  DELIVERY_ASCEND_PICK = 11,
  DELIVERY_GOTO_DROP = 12,
  DELIVERY_DESCEND_DROP = 13,
  DELIVERY_DROP_HOLD = 14,
  DELIVERY_ASCEND_DROP = 15,
};

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publishTarget(const Target & target);
  void publishActiveController(std::uint8_t controller_id);
  void publishVisualTakeoverState(bool active);
  void publishHeightFilterEnabled(bool enabled);
  void publishPillarTaskResult(const PillarTaskResult & result);
  void publishPillarTaskSummary();
  void logMissionSummary() const;

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;
  double normalizeAngleDeg(double angle_deg) const;
  bool hasFreshHoverVisualData(const rclcpp::Time & reference_time) const;
  bool isHoverVisuallyAligned() const;

  void pillarPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void groundHeightCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void minRangeCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void circleRankCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void circleCenterCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void monitorTimerCallback();

  void startHoverMeasurement();
  void resetCurrentPillarSamples();
  void sampleHoverMeasurement();
  void pruneHoverCircleRankSamples(const rclcpp::Time & reference_time);
  void finishCurrentPillarMeasurement();
  void completeMission(const std::string & reason);

  // Delivery phase helpers
  void startDeliveryPhase();
  void startGoToLanding();
  bool planDeliveryOrder();
  void beginDeliveryPickup();
  void beginDeliveryDrop();
  void advanceDeliveryStep();
  void publishMagnetCommand(bool energize);
  void publishServoCommand(bool down);
  double pillarDescendTargetCm(std::size_t pillar_index, double clearance_cm) const;

  static double meterToCm(double value_m);
  static double cmToMeter(double value_cm);
  static double radToDeg(double value_rad);

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_takeover_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pillar_task_result_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pillar_task_summary_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr on_pillar_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr height_filter_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr magnet_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_command_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ground_height_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr min_range_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr circle_rank_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pillar_position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr circle_center_sub_;
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
  double height_band_cm_;
  double landing_x_cm_;
  double landing_y_cm_;
  double mission_yaw_deg_;
  double hover_time_sec_;
  double pillar_height_threshold_cm_;
  double landing_align_time_sec_;
  double visual_alignment_tolerance_px_;
  double visual_alignment_timeout_sec_;

  // Delivery phase parameters
  bool delivery_phase_enable_;
  double pickup_clearance_cm_;
  double drop_clearance_cm_;
  double pickup_hold_sec_;
  double drop_hold_sec_;
  int pickup_retry_max_;
  double delivery_align_settle_sec_;

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
  std::vector<PillarTaskSummarySlot> pillar_summary_slots_;

  rclcpp::Time hover_start_time_;
  rclcpp::Time hover_last_visual_sample_time_;
  struct TimedCircleRankSample
  {
    rclcpp::Time stamp;
    int rank;
  };
  bool pillar_ground_height_valid_;
  double pillar_ground_height_max_m_;
  bool pillar_min_range_valid_;
  double pillar_min_range_min_m_;
  std::size_t pillar_ground_height_sample_count_;
  std::size_t pillar_min_range_sample_count_;
  std::deque<TimedCircleRankSample> hover_circle_rank_samples_;
  double hover_visual_error_x_px_;
  double hover_visual_error_y_px_;
  bool hover_has_visual_alignment_;

  rclcpp::Time land_align_start_time_;

  // Delivery phase state
  std::size_t empty_pillar_index_;
  bool empty_pillar_valid_;
  std::vector<std::size_t> delivery_pickup_order_;  // pillar indices for rank 1, 2, 3
  std::size_t delivery_step_;                       // 0..delivery_pickup_order_.size()
  int pickup_retry_count_;
  rclcpp::Time delivery_phase_start_time_;
  rclcpp::Time delivery_align_start_time_;
  bool delivery_align_started_;

  void publishOnPillar(bool active);
};

}  // namespace activity_control_pkg
