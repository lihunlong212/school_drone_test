#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

namespace
{
constexpr double kDefaultTimerPeriodSec = 0.05;
constexpr double kCruiseAltitudeThresholdCm = 20.0;
constexpr std::uint8_t kSummaryStatusPending = 0;
constexpr std::uint8_t kSummaryStatusMeasuring = 1;
constexpr std::uint8_t kSummaryStatusDone = 2;
constexpr std::uint8_t kSummaryStatusInvalid = 3;
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  pos_tol_cm_(declare_parameter("position_tolerance_cm", 6.0)),
  yaw_tol_deg_(declare_parameter("yaw_tolerance_deg", 5.0)),
  height_tol_cm_(declare_parameter("height_tolerance_cm", 6.0)),
  start_x_cm_(declare_parameter("start_x_cm", 0.0)),
  start_y_cm_(declare_parameter("start_y_cm", 0.0)),
  cruise_height_cm_(declare_parameter("cruise_height_cm", 130.0)),
  landing_x_cm_(declare_parameter("landing_x_cm", 0.0)),
  landing_y_cm_(declare_parameter("landing_y_cm", 0.0)),
  mission_yaw_deg_(declare_parameter("mission_yaw_deg", 0.0)),
  hover_time_sec_(declare_parameter("hover_time_sec", 5.0)),
  pillar_height_threshold_cm_(declare_parameter("pillar_height_threshold_cm", 40.0)),
  has_height_(false),
  current_height_cm_(0.0),
  latest_circle_rank_(0),
  phase_(MissionPhase::WAIT_PILLARS),
  pillars_received_(false),
  mission_complete_sent_(false),
  has_current_target_(false),
  current_target_{0.0, 0.0, 0.0, 0.0},
  current_pillar_index_(0),
  hover_height_sum_cm_(0.0),
  hover_height_sample_count_(0)
{
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  visual_takeover_active_pub_ =
    create_publisher<std_msgs::msg::Bool>("/visual_takeover_active", durable_qos);
  mission_complete_pub_ =
    create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());
  pillar_task_result_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>("/pillar_task_result", durable_qos);
  pillar_task_summary_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>("/pillar_task_summary", durable_qos);
  fine_data_pub_ =
    create_publisher<std_msgs::msg::Int32MultiArray>("/fine_data", rclcpp::QoS(10));

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));
  circle_rank_sub_ = create_subscription<std_msgs::msg::Int32>(
    "/circle_rank",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::circleRankCallback, this, std::placeholders::_1));
  pillar_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/pillar_position",
    durable_qos,
    std::bind(&RouteTargetPublisherNode::pillarPositionCallback, this, std::placeholders::_1));
  circle_center_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/circle_center",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::circleCenterCallback, this, std::placeholders::_1));

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  publishVisualTakeoverState(false);
  publishActiveController(3);

  RCLCPP_INFO(
    get_logger(),
    "RouteTargetPublisher initialized: waiting for /pillar_position, map=%s laser_link=%s output=%s",
    map_frame_.c_str(),
    laser_link_frame_.c_str(),
    output_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Mission params: start=(%.1f, %.1f)cm cruise=%.1fcm landing=(%.1f, %.1f)cm yaw=%.1fdeg hover=%.1fs threshold=%.1fcm",
    start_x_cm_,
    start_y_cm_,
    cruise_height_cm_,
    landing_x_cm_,
    landing_y_cm_,
    mission_yaw_deg_,
    hover_time_sec_,
    pillar_height_threshold_cm_);
}

void RouteTargetPublisherNode::publishTarget(const Target & target)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);
  target_pub_->publish(message);

  publishActiveController(2);

  RCLCPP_INFO(
    get_logger(),
    "Published target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg phase=%u",
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.yaw_deg,
    static_cast<unsigned>(phase_));
}

void RouteTargetPublisherNode::publishActiveController(std::uint8_t controller_id)
{
  std_msgs::msg::UInt8 message;
  message.data = controller_id;
  active_controller_pub_->publish(message);
}

void RouteTargetPublisherNode::publishVisualTakeoverState(bool active)
{
  std_msgs::msg::Bool message;
  message.data = active;
  visual_takeover_active_pub_->publish(message);
}

void RouteTargetPublisherNode::publishPillarTaskResult(const PillarTaskResult & result)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(6);
  message.data[0] = static_cast<float>(result.pillar_index);
  message.data[1] = static_cast<float>(result.x_m);
  message.data[2] = static_cast<float>(result.y_m);
  message.data[3] = static_cast<float>(result.pillar_height_cm);
  message.data[4] = static_cast<float>(result.circle_rank);
  message.data[5] = result.valid ? 1.0F : 0.0F;
  pillar_task_result_pub_->publish(message);
}

void RouteTargetPublisherNode::publishPillarTaskSummary()
{
  std_msgs::msg::Float32MultiArray message;
  message.data.reserve(pillar_summary_slots_.size() * 6);

  for (const auto & slot : pillar_summary_slots_) {
    message.data.push_back(static_cast<float>(slot.pillar_index));
    message.data.push_back(static_cast<float>(slot.x_m));
    message.data.push_back(static_cast<float>(slot.y_m));
    message.data.push_back(static_cast<float>(slot.pillar_height_cm));
    message.data.push_back(static_cast<float>(slot.circle_rank));
    message.data.push_back(static_cast<float>(slot.status));
  }

  pillar_task_summary_pub_->publish(message);
}

void RouteTargetPublisherNode::logMissionSummary() const
{
  if (pillar_summary_slots_.empty()) {
    RCLCPP_INFO(get_logger(), "Mission summary: no pillar results recorded.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Mission summary: %zu pillar slots recorded.", pillar_summary_slots_.size());
  for (const auto & slot : pillar_summary_slots_) {
    const char * status_text = "pending";
    switch (slot.status) {
      case kSummaryStatusMeasuring:
        status_text = "measuring";
        break;
      case kSummaryStatusDone:
        status_text = "done";
        break;
      case kSummaryStatusInvalid:
        status_text = "invalid";
        break;
      case kSummaryStatusPending:
      default:
        status_text = "pending";
        break;
    }

    RCLCPP_INFO(
      get_logger(),
      "Pillar %zu: x=%.3fm y=%.3fm height=%.1fcm circle_rank=%d status=%s",
      slot.pillar_index,
      slot.x_m,
      slot.y_m,
      slot.pillar_height_cm,
      slot.circle_rank,
      status_text);
  }
}

bool RouteTargetPublisherNode::getCurrentPose(
  double & x_cm,
  double & y_cm,
  double & z_cm,
  double & yaw_deg)
{
  if (!has_height_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Waiting for /height before evaluating mission progress.");
    return false;
  }

  try {
    const geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);

    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = current_height_cm_;

    tf2::Quaternion quaternion;
    tf2::fromMsg(transform.transform.rotation, quaternion);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(),
      laser_link_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool RouteTargetPublisherNode::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dz = target.z_cm - z_cm;
  const double dxy = std::hypot(dx, dy);
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);

  const bool xy_ok = dxy <= pos_tol_cm_;
  const bool z_ok = std::fabs(dz) <= height_tol_cm_;

  if (target.z_cm > kCruiseAltitudeThresholdCm) {
    return xy_ok && z_ok;
  }

  return xy_ok && z_ok && std::fabs(dyaw) <= yaw_tol_deg_;
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}

void RouteTargetPublisherNode::pillarPositionCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (pillars_received_ || phase_ != MissionPhase::WAIT_PILLARS) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Ignoring /pillar_position because the mission has already started.");
    return;
  }

  if (msg->data.size() % 2 != 0) {
    RCLCPP_WARN(get_logger(), "/pillar_position requires an even number of values [x1,y1,...]");
    return;
  }

  pillar_targets_.clear();
  pillar_results_.clear();
  pillar_summary_slots_.clear();
  current_pillar_index_ = 0;

  for (std::size_t index = 0; index < msg->data.size(); index += 2) {
    const double x_m = static_cast<double>(msg->data[index]);
    const double y_m = static_cast<double>(msg->data[index + 1]);
    pillar_targets_.push_back(Target{
      meterToCm(x_m),
      meterToCm(y_m),
      cruise_height_cm_,
      mission_yaw_deg_,
    });
  }

  std::sort(
    pillar_targets_.begin(),
    pillar_targets_.end(),
    [](const Target & lhs, const Target & rhs) {
      return std::hypot(lhs.x_cm, lhs.y_cm) < std::hypot(rhs.x_cm, rhs.y_cm);
    });

  pillars_received_ = true;

  if (pillar_targets_.empty()) {
    completeMission("Received /pillar_position, but no valid pillars were provided.");
    return;
  }

  pillar_summary_slots_.reserve(pillar_targets_.size());
  for (std::size_t index = 0; index < pillar_targets_.size(); ++index) {
    const auto & target = pillar_targets_[index];
    pillar_summary_slots_.push_back(PillarTaskSummarySlot{
      index + 1,
      cmToMeter(target.x_cm),
      cmToMeter(target.y_cm),
      0.0,
      0,
      kSummaryStatusPending,
    });
  }

  phase_ = MissionPhase::TAKEOFF_TO_CRUISE;
  has_current_target_ = true;
  current_target_ = Target{start_x_cm_, start_y_cm_, cruise_height_cm_, mission_yaw_deg_};

  RCLCPP_INFO(get_logger(), "Received %zu pillar targets. Mission will start automatically.", pillar_targets_.size());
  for (std::size_t index = 0; index < pillar_targets_.size(); ++index) {
    const auto & target = pillar_targets_[index];
    RCLCPP_INFO(
      get_logger(),
      "Pillar order %zu: x=%.3fm y=%.3fm distance=%.3fm",
      index + 1,
      cmToMeter(target.x_cm),
      cmToMeter(target.y_cm),
      std::hypot(cmToMeter(target.x_cm), cmToMeter(target.y_cm)));
  }

  publishPillarTaskSummary();
  publishTarget(current_target_);
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void RouteTargetPublisherNode::circleRankCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_circle_rank_ = msg->data;
}

void RouteTargetPublisherNode::circleCenterCallback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (phase_ != MissionPhase::HOVER_AND_MEASURE || !fine_data_pub_) {
    return;
  }

  std_msgs::msg::Int32MultiArray fine_data_msg;
  fine_data_msg.data.resize(2);
  fine_data_msg.data[0] = static_cast<std::int32_t>(std::lround(msg->point.x));
  fine_data_msg.data[1] = static_cast<std::int32_t>(std::lround(msg->point.y));
  fine_data_pub_->publish(fine_data_msg);

  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    *get_clock(),
    500,
    "Forwarding /circle_center to /fine_data during hover: x=%d y=%d",
    fine_data_msg.data[0],
    fine_data_msg.data[1]);
}

void RouteTargetPublisherNode::startHoverMeasurement()
{
  phase_ = MissionPhase::HOVER_AND_MEASURE;
  hover_start_time_ = now();
  hover_height_sum_cm_ = 0.0;
  hover_height_sample_count_ = 0;
  hover_circle_rank_counts_.clear();
  publishVisualTakeoverState(true);

  if (current_pillar_index_ < pillar_summary_slots_.size()) {
    pillar_summary_slots_[current_pillar_index_].status = kSummaryStatusMeasuring;
    publishPillarTaskSummary();
  }

  RCLCPP_INFO(
    get_logger(),
    "Reached pillar %zu/%zu. Starting %.1fs hover measurement at x=%.3fm y=%.3fm.",
    current_pillar_index_ + 1,
    pillar_targets_.size(),
    hover_time_sec_,
    cmToMeter(current_target_.x_cm),
    cmToMeter(current_target_.y_cm));
}

void RouteTargetPublisherNode::sampleHoverMeasurement()
{
  if (has_height_) {
    hover_height_sum_cm_ += current_height_cm_;
    ++hover_height_sample_count_;
  }

  if (latest_circle_rank_ > 0) {
    ++hover_circle_rank_counts_[latest_circle_rank_];
  }
}

void RouteTargetPublisherNode::finishCurrentPillarMeasurement()
{
  publishVisualTakeoverState(false);

  const double avg_height_cm = hover_height_sample_count_ > 0
    ? (hover_height_sum_cm_ / static_cast<double>(hover_height_sample_count_))
    : cruise_height_cm_;
  const double raw_pillar_height_cm = cruise_height_cm_ - avg_height_cm;
  const bool valid =
    hover_height_sample_count_ > 0 && raw_pillar_height_cm > pillar_height_threshold_cm_;

  int mode_circle_rank = 0;
  int mode_count = 0;
  for (const auto & entry : hover_circle_rank_counts_) {
    if (entry.second > mode_count || (entry.second == mode_count && entry.first < mode_circle_rank)) {
      mode_circle_rank = entry.first;
      mode_count = entry.second;
    }
  }

  const Target & pillar_target = pillar_targets_[current_pillar_index_];
  const PillarTaskResult result{
    current_pillar_index_ + 1,
    cmToMeter(pillar_target.x_cm),
    cmToMeter(pillar_target.y_cm),
    valid ? raw_pillar_height_cm : 0.0,
    mode_circle_rank,
    valid,
  };

  pillar_results_.push_back(result);
  publishPillarTaskResult(result);

  if (current_pillar_index_ < pillar_summary_slots_.size()) {
    auto & slot = pillar_summary_slots_[current_pillar_index_];
    slot.pillar_height_cm = result.pillar_height_cm;
    slot.circle_rank = result.circle_rank;
    slot.status = result.valid ? kSummaryStatusDone : kSummaryStatusInvalid;
    publishPillarTaskSummary();
  }

  RCLCPP_INFO(
    get_logger(),
    "Finished pillar %zu/%zu: avg_height=%.1fcm pillar_height=%.1fcm circle_rank=%d valid=%s samples=%zu",
    result.pillar_index,
    pillar_targets_.size(),
    avg_height_cm,
    result.pillar_height_cm,
    result.circle_rank,
    result.valid ? "true" : "false",
    hover_height_sample_count_);

  ++current_pillar_index_;
  if (current_pillar_index_ < pillar_targets_.size()) {
    phase_ = MissionPhase::VISIT_PILLAR;
    current_target_ = pillar_targets_[current_pillar_index_];
    publishTarget(current_target_);
    return;
  }

  phase_ = MissionPhase::GO_TO_LAND_POINT;
  current_target_ = Target{landing_x_cm_, landing_y_cm_, cruise_height_cm_, mission_yaw_deg_};
  publishTarget(current_target_);
}

void RouteTargetPublisherNode::completeMission(const std::string & reason)
{
  if (phase_ == MissionPhase::COMPLETE) {
    return;
  }

  phase_ = MissionPhase::COMPLETE;
  has_current_target_ = false;
  publishVisualTakeoverState(false);
  publishActiveController(3);

  if (!mission_complete_sent_) {
    std_msgs::msg::Empty message;
    mission_complete_pub_->publish(message);
    mission_complete_sent_ = true;
  }

  publishPillarTaskSummary();
  RCLCPP_INFO(get_logger(), "%s", reason.c_str());
  logMissionSummary();
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (phase_ == MissionPhase::WAIT_PILLARS) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for /pillar_position before starting the mission.");
    return;
  }

  if (phase_ == MissionPhase::COMPLETE) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Mission already completed. Keeping controller in stop state.");
    return;
  }

  if (phase_ == MissionPhase::HOVER_AND_MEASURE) {
    sampleHoverMeasurement();

    const double elapsed_sec = (now() - hover_start_time_).seconds();
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Hovering on pillar %zu/%zu: elapsed=%.1fs/%.1fs latest_circle_rank=%d samples=%zu",
      current_pillar_index_ + 1,
      pillar_targets_.size(),
      elapsed_sec,
      hover_time_sec_,
      latest_circle_rank_,
      hover_height_sample_count_);

    if (elapsed_sec >= hover_time_sec_) {
      finishCurrentPillarMeasurement();
    }
    return;
  }

  if (!has_current_target_) {
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "Tracking target phase=%u target=(%.1f, %.1f, %.1f, %.1f) current=(%.1f, %.1f, %.1f, %.1f)",
    static_cast<unsigned>(phase_),
    current_target_.x_cm,
    current_target_.y_cm,
    current_target_.z_cm,
    current_target_.yaw_deg,
    x_cm,
    y_cm,
    z_cm,
    yaw_deg);

  if (!isReached(current_target_, x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  switch (phase_) {
    case MissionPhase::TAKEOFF_TO_CRUISE:
      phase_ = MissionPhase::VISIT_PILLAR;
      current_target_ = pillar_targets_[current_pillar_index_];
      publishTarget(current_target_);
      break;

    case MissionPhase::VISIT_PILLAR:
      startHoverMeasurement();
      break;

    case MissionPhase::GO_TO_LAND_POINT:
      phase_ = MissionPhase::LAND;
      current_target_ = Target{landing_x_cm_, landing_y_cm_, 0.0, mission_yaw_deg_};
      publishTarget(current_target_);
      break;

    case MissionPhase::LAND:
      completeMission("Mission finished. Landing target reached and mission_complete published.");
      break;

    case MissionPhase::WAIT_PILLARS:
    case MissionPhase::HOVER_AND_MEASURE:
    case MissionPhase::COMPLETE:
      break;
  }
}

double RouteTargetPublisherNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double RouteTargetPublisherNode::cmToMeter(double value_cm)
{
  return value_cm / 100.0;
}

double RouteTargetPublisherNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

}  // namespace activity_control_pkg
