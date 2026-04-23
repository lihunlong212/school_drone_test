#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
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
constexpr double kCircleRankWindowSec = 5.0;
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
  height_band_cm_(declare_parameter("height_band_cm", 20.0)),
  landing_x_cm_(declare_parameter("landing_x_cm", 250.0)),
  landing_y_cm_(declare_parameter("landing_y_cm", -250.0)),
  mission_yaw_deg_(declare_parameter("mission_yaw_deg", 0.0)),
  hover_time_sec_(declare_parameter("hover_time_sec", 5.0)),
  pillar_height_threshold_cm_(declare_parameter("pillar_height_threshold_cm", 40.0)),
  landing_align_time_sec_(declare_parameter("landing_align_time_sec", 3.0)),
  visual_alignment_tolerance_px_(declare_parameter("visual_alignment_tolerance_px", 20.0)),
  visual_alignment_timeout_sec_(declare_parameter("visual_alignment_timeout_sec", 0.5)),
  delivery_phase_enable_(declare_parameter("delivery_phase_enable", false)),
  pickup_clearance_cm_(declare_parameter("pickup_clearance_cm", 20.0)),
  drop_clearance_cm_(declare_parameter("drop_clearance_cm", 20.0)),
  pickup_hold_sec_(declare_parameter("pickup_hold_sec", 3.0)),
  drop_hold_sec_(declare_parameter("drop_hold_sec", 1.0)),
  pickup_retry_max_(static_cast<int>(declare_parameter("pickup_retry_max", 3))),
  delivery_align_settle_sec_(declare_parameter("delivery_align_settle_sec", 1.5)),
  has_height_(false),
  current_height_cm_(0.0),
  latest_circle_rank_(0),
  phase_(MissionPhase::WAIT_PILLARS),
  pillars_received_(false),
  mission_complete_sent_(false),
  has_current_target_(false),
  current_target_{0.0, 0.0, 0.0, 0.0},
  current_pillar_index_(0),
  pillar_ground_height_valid_(false),
  pillar_ground_height_max_m_(0.0),
  pillar_min_range_valid_(false),
  pillar_min_range_min_m_(0.0),
  pillar_ground_height_sample_count_(0),
  pillar_min_range_sample_count_(0),
  hover_visual_error_x_px_(0.0),
  hover_visual_error_y_px_(0.0),
  hover_has_visual_alignment_(false),
  empty_pillar_index_(0),
  empty_pillar_valid_(false),
  delivery_step_(0),
  pickup_retry_count_(0),
  delivery_align_started_(false)
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
  on_pillar_pub_ =
    create_publisher<std_msgs::msg::Bool>("/on_pillar", durable_qos);
  height_filter_enabled_pub_ =
    create_publisher<std_msgs::msg::Bool>("/height_filter_enabled", durable_qos);
  magnet_command_pub_ =
    create_publisher<std_msgs::msg::Bool>("/magnet_command", durable_qos);
  servo_command_pub_ =
    create_publisher<std_msgs::msg::Bool>("/servo_command", rclcpp::QoS(10).reliable());

  ground_height_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/laser_array/ground_height",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::groundHeightCallback, this, std::placeholders::_1));
  min_range_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/laser_array/min_range",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::minRangeCallback, this, std::placeholders::_1));
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
  publishOnPillar(false);
  publishHeightFilterEnabled(false);
  publishActiveController(3);
  publishMagnetCommand(false);
  publishServoCommand(false);

  RCLCPP_INFO(
    get_logger(),
    "RouteTargetPublisher initialized: waiting for /pillar_position, map=%s laser_link=%s output=%s",
    map_frame_.c_str(),
    laser_link_frame_.c_str(),
    output_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Mission params: start=(%.1f, %.1f)cm cruise=%.1fcm band=%.1fcm landing=(%.1f, %.1f)cm yaw=%.1fdeg hover=%.1fs",
    start_x_cm_,
    start_y_cm_,
    cruise_height_cm_,
    height_band_cm_,
    landing_x_cm_,
    landing_y_cm_,
    mission_yaw_deg_,
    hover_time_sec_);
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

void RouteTargetPublisherNode::publishHeightFilterEnabled(bool enabled)
{
  if (!height_filter_enabled_pub_) {
    return;
  }

  std_msgs::msg::Bool message;
  message.data = enabled;
  height_filter_enabled_pub_->publish(message);
}

void RouteTargetPublisherNode::publishOnPillar(bool active)
{
  if (!on_pillar_pub_) {
    return;
  }
  std_msgs::msg::Bool message;
  message.data = active;
  on_pillar_pub_->publish(message);
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
      "Waiting for /laser_array/ground_height before evaluating mission progress.");
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

bool RouteTargetPublisherNode::hasFreshHoverVisualData(const rclcpp::Time & reference_time) const
{
  if (!hover_has_visual_alignment_ || hover_last_visual_sample_time_.nanoseconds() == 0) {
    return false;
  }

  return (reference_time - hover_last_visual_sample_time_).seconds() <= visual_alignment_timeout_sec_;
}

bool RouteTargetPublisherNode::isHoverVisuallyAligned() const
{
  return
    std::fabs(hover_visual_error_x_px_) <= visual_alignment_tolerance_px_ &&
    std::fabs(hover_visual_error_y_px_) <= visual_alignment_tolerance_px_;
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
  resetCurrentPillarSamples();
  publishHeightFilterEnabled(false);

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

void RouteTargetPublisherNode::groundHeightCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Ignoring non-finite /laser_array/ground_height sample.");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const double ground_height_m = static_cast<double>(msg->data);
  current_height_cm_ = meterToCm(ground_height_m);
  has_height_ = true;

  if (phase_ != MissionPhase::HOVER_AND_MEASURE || current_pillar_index_ >= pillar_targets_.size()) {
    return;
  }

  if (!pillar_ground_height_valid_ || ground_height_m > pillar_ground_height_max_m_) {
    pillar_ground_height_max_m_ = ground_height_m;
  }
  pillar_ground_height_valid_ = true;
  ++pillar_ground_height_sample_count_;
}

void RouteTargetPublisherNode::minRangeCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Ignoring non-finite /laser_array/min_range sample.");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != MissionPhase::HOVER_AND_MEASURE || current_pillar_index_ >= pillar_targets_.size()) {
    return;
  }

  const double min_range_m = static_cast<double>(msg->data);
  if (!pillar_min_range_valid_ || min_range_m < pillar_min_range_min_m_) {
    pillar_min_range_min_m_ = min_range_m;
  }
  pillar_min_range_valid_ = true;
  ++pillar_min_range_sample_count_;
}

void RouteTargetPublisherNode::circleRankCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_circle_rank_ = msg->data;

  if (phase_ != MissionPhase::HOVER_AND_MEASURE || current_pillar_index_ >= pillar_targets_.size()) {
    return;
  }

  const rclcpp::Time sample_time = now();
  pruneHoverCircleRankSamples(sample_time);
  if (msg->data > 0 && hasFreshHoverVisualData(sample_time) && isHoverVisuallyAligned()) {
    hover_circle_rank_samples_.push_back(TimedCircleRankSample{sample_time, msg->data});
  }
}

void RouteTargetPublisherNode::circleCenterCallback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const bool forward_phase =
    phase_ == MissionPhase::HOVER_AND_MEASURE ||
    phase_ == MissionPhase::LAND_ALIGN ||
    phase_ == MissionPhase::LAND ||
    phase_ == MissionPhase::DELIVERY_DESCEND_PICK ||
    phase_ == MissionPhase::DELIVERY_PICK_HOLD ||
    phase_ == MissionPhase::DELIVERY_DESCEND_DROP ||
    phase_ == MissionPhase::DELIVERY_DROP_HOLD;

  if (!forward_phase || !fine_data_pub_) {
    return;
  }

  if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y)) {
    return;
  }

  std_msgs::msg::Int32MultiArray fine_data_msg;
  fine_data_msg.data.resize(2);
  fine_data_msg.data[0] = static_cast<std::int32_t>(std::lround(msg->point.x));
  fine_data_msg.data[1] = static_cast<std::int32_t>(std::lround(msg->point.y));
  fine_data_pub_->publish(fine_data_msg);

  if (phase_ == MissionPhase::HOVER_AND_MEASURE ||
      phase_ == MissionPhase::DELIVERY_DESCEND_PICK ||
      phase_ == MissionPhase::DELIVERY_PICK_HOLD ||
      phase_ == MissionPhase::DELIVERY_DESCEND_DROP ||
      phase_ == MissionPhase::DELIVERY_DROP_HOLD) {
    hover_visual_error_x_px_ = msg->point.x;
    hover_visual_error_y_px_ = msg->point.y;
    hover_last_visual_sample_time_ = now();
    hover_has_visual_alignment_ = true;
  }

  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    *get_clock(),
    500,
    "Forwarding /circle_center to /fine_data (phase=%u): x=%d y=%d",
    static_cast<unsigned>(phase_),
    fine_data_msg.data[0],
    fine_data_msg.data[1]);
}

void RouteTargetPublisherNode::resetCurrentPillarSamples()
{
  pillar_ground_height_valid_ = false;
  pillar_ground_height_max_m_ = 0.0;
  pillar_min_range_valid_ = false;
  pillar_min_range_min_m_ = 0.0;
  pillar_ground_height_sample_count_ = 0;
  pillar_min_range_sample_count_ = 0;
}

void RouteTargetPublisherNode::startHoverMeasurement()
{
  phase_ = MissionPhase::HOVER_AND_MEASURE;
  hover_start_time_ = now();
  resetCurrentPillarSamples();
  hover_circle_rank_samples_.clear();
  hover_last_visual_sample_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  hover_visual_error_x_px_ = 0.0;
  hover_visual_error_y_px_ = 0.0;
  hover_has_visual_alignment_ = false;
  publishVisualTakeoverState(true);
  publishOnPillar(true);
  publishHeightFilterEnabled(true);

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
  pruneHoverCircleRankSamples(now());
}

void RouteTargetPublisherNode::pruneHoverCircleRankSamples(const rclcpp::Time & reference_time)
{
  while (!hover_circle_rank_samples_.empty()) {
    const double age_sec = (reference_time - hover_circle_rank_samples_.front().stamp).seconds();
    if (age_sec <= kCircleRankWindowSec) {
      break;
    }
    hover_circle_rank_samples_.pop_front();
  }
}

void RouteTargetPublisherNode::finishCurrentPillarMeasurement()
{
  publishOnPillar(false);
  publishVisualTakeoverState(false);
  pruneHoverCircleRankSamples(now());

  const bool valid = pillar_ground_height_valid_ && pillar_min_range_valid_;
  const double pillar_height_cm = valid
    ? std::max(0.0, pillar_ground_height_max_m_ - pillar_min_range_min_m_) * 100.0
    : 0.0;

  std::map<int, int> hover_circle_rank_counts;
  for (const auto & sample : hover_circle_rank_samples_) {
    ++hover_circle_rank_counts[sample.rank];
  }

  int mode_circle_rank = 0;
  int mode_count = 0;
  for (const auto & entry : hover_circle_rank_counts) {
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
    pillar_height_cm,
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
    "Finished pillar %zu/%zu: pillar_height=%.1fcm circle_rank=%d valid=%s ground_samples=%zu min_samples=%zu aligned_rank_samples_in_5s=%zu visual_aligned_now=%s visual_err=(%.1f, %.1f)px",
    result.pillar_index,
    pillar_targets_.size(),
    pillar_height_cm,
    result.circle_rank,
    result.valid ? "true" : "false",
    pillar_ground_height_sample_count_,
    pillar_min_range_sample_count_,
    hover_circle_rank_samples_.size(),
    (hasFreshHoverVisualData(now()) && isHoverVisuallyAligned()) ? "true" : "false",
    hover_visual_error_x_px_,
    hover_visual_error_y_px_);

  ++current_pillar_index_;
  if (current_pillar_index_ < pillar_targets_.size()) {
    phase_ = MissionPhase::VISIT_PILLAR;
    current_target_ = pillar_targets_[current_pillar_index_];
    resetCurrentPillarSamples();
    publishHeightFilterEnabled(true);
    publishTarget(current_target_);
    return;
  }

  if (delivery_phase_enable_) {
    if (planDeliveryOrder()) {
      startDeliveryPhase();
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "delivery_phase_enable=true but pillar plan invalid (need 1 empty + ranks {1,2,3}); falling back to landing.");
  }

  startGoToLanding();
}

void RouteTargetPublisherNode::startGoToLanding()
{
  phase_ = MissionPhase::GO_TO_LAND_POINT;
  publishHeightFilterEnabled(false);
  publishVisualTakeoverState(false);
  publishOnPillar(false);
  publishMagnetCommand(false);
  publishServoCommand(false);
  current_target_ = Target{landing_x_cm_, landing_y_cm_, cruise_height_cm_, mission_yaw_deg_};
  publishTarget(current_target_);
}

bool RouteTargetPublisherNode::planDeliveryOrder()
{
  empty_pillar_valid_ = false;
  delivery_pickup_order_.clear();

  std::vector<std::size_t> rank_to_pillar(4, std::numeric_limits<std::size_t>::max());
  std::size_t empty_candidate = std::numeric_limits<std::size_t>::max();

  for (std::size_t i = 0; i < pillar_summary_slots_.size(); ++i) {
    const auto & slot = pillar_summary_slots_[i];
    if (slot.status != kSummaryStatusDone) {
      continue;
    }
    const int rank = slot.circle_rank;
    if (rank == 0) {
      if (empty_candidate == std::numeric_limits<std::size_t>::max()) {
        empty_candidate = i;
      }
    } else if (rank >= 1 && rank <= 3) {
      if (rank_to_pillar[rank] == std::numeric_limits<std::size_t>::max()) {
        rank_to_pillar[rank] = i;
      }
    }
  }

  if (empty_candidate == std::numeric_limits<std::size_t>::max()) {
    return false;
  }
  for (int r = 1; r <= 3; ++r) {
    if (rank_to_pillar[r] == std::numeric_limits<std::size_t>::max()) {
      return false;
    }
  }

  empty_pillar_index_ = empty_candidate;
  empty_pillar_valid_ = true;
  delivery_pickup_order_ = {rank_to_pillar[1], rank_to_pillar[2], rank_to_pillar[3]};
  delivery_step_ = 0;
  pickup_retry_count_ = 0;

  RCLCPP_INFO(
    get_logger(),
    "Delivery plan: empty=%zu, pickup order (rank 1->2->3): %zu, %zu, %zu",
    empty_pillar_index_ + 1,
    delivery_pickup_order_[0] + 1,
    delivery_pickup_order_[1] + 1,
    delivery_pickup_order_[2] + 1);

  return true;
}

void RouteTargetPublisherNode::startDeliveryPhase()
{
  delivery_phase_start_time_ = now();
  beginDeliveryPickup();
}

void RouteTargetPublisherNode::beginDeliveryPickup()
{
  if (delivery_step_ >= delivery_pickup_order_.size()) {
    startGoToLanding();
    return;
  }

  const std::size_t pillar_idx = delivery_pickup_order_[delivery_step_];
  const Target & pillar = pillar_targets_[pillar_idx];

  phase_ = MissionPhase::DELIVERY_GOTO_PICK;
  pickup_retry_count_ = 0;
  delivery_align_started_ = false;
  publishHeightFilterEnabled(false);
  publishVisualTakeoverState(false);
  publishOnPillar(false);
  publishMagnetCommand(false);
  publishServoCommand(false);

  current_target_ = Target{pillar.x_cm, pillar.y_cm, cruise_height_cm_, mission_yaw_deg_};

  RCLCPP_INFO(
    get_logger(),
    "Delivery step %zu/%zu: GOTO_PICK pillar=%zu rank=%d",
    delivery_step_ + 1,
    delivery_pickup_order_.size(),
    pillar_idx + 1,
    static_cast<int>(delivery_step_ + 1));

  publishTarget(current_target_);
}

void RouteTargetPublisherNode::beginDeliveryDrop()
{
  if (!empty_pillar_valid_) {
    startGoToLanding();
    return;
  }

  const Target & empty_target = pillar_targets_[empty_pillar_index_];
  phase_ = MissionPhase::DELIVERY_GOTO_DROP;
  delivery_align_started_ = false;
  publishHeightFilterEnabled(false);
  publishVisualTakeoverState(false);
  publishOnPillar(false);

  current_target_ = Target{empty_target.x_cm, empty_target.y_cm, cruise_height_cm_, mission_yaw_deg_};

  RCLCPP_INFO(
    get_logger(),
    "Delivery step %zu/%zu: GOTO_DROP pillar=%zu",
    delivery_step_ + 1,
    delivery_pickup_order_.size(),
    empty_pillar_index_ + 1);

  publishTarget(current_target_);
}

void RouteTargetPublisherNode::advanceDeliveryStep()
{
  ++delivery_step_;
  beginDeliveryPickup();
}

void RouteTargetPublisherNode::publishMagnetCommand(bool energize)
{
  if (!magnet_command_pub_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = energize;
  magnet_command_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Magnet command -> %s", energize ? "ON" : "OFF");
}

void RouteTargetPublisherNode::publishServoCommand(bool down)
{
  if (!servo_command_pub_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = down;
  servo_command_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Servo command -> %s", down ? "DOWN" : "UP");
}

double RouteTargetPublisherNode::pillarDescendTargetCm(
  std::size_t pillar_index, double clearance_cm) const
{
  double pillar_height_cm = 0.0;
  if (pillar_index < pillar_summary_slots_.size()) {
    pillar_height_cm = pillar_summary_slots_[pillar_index].pillar_height_cm;
  }
  if (pillar_height_cm <= 0.0) {
    pillar_height_cm = pillar_height_threshold_cm_;
  }
  return pillar_height_cm + clearance_cm;
}

void RouteTargetPublisherNode::completeMission(const std::string & reason)
{
  if (phase_ == MissionPhase::COMPLETE) {
    return;
  }

  phase_ = MissionPhase::COMPLETE;
  has_current_target_ = false;
  publishVisualTakeoverState(false);
  publishOnPillar(false);
  publishHeightFilterEnabled(false);
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
      "Hovering on pillar %zu/%zu: elapsed=%.1fs/%.1fs latest_circle_rank=%d ground_samples=%zu min_samples=%zu visual_aligned=%s visual_err=(%.1f, %.1f)px aligned_rank_samples=%zu",
      current_pillar_index_ + 1,
      pillar_targets_.size(),
      elapsed_sec,
      hover_time_sec_,
      latest_circle_rank_,
      pillar_ground_height_sample_count_,
      pillar_min_range_sample_count_,
      (hasFreshHoverVisualData(now()) && isHoverVisuallyAligned()) ? "true" : "false",
      hover_visual_error_x_px_,
      hover_visual_error_y_px_,
      hover_circle_rank_samples_.size());

    if (elapsed_sec >= hover_time_sec_) {
      finishCurrentPillarMeasurement();
    }
    return;
  }

  // Delivery time-based phases (run independent of target-reach checks)
  if (phase_ == MissionPhase::DELIVERY_PICK_HOLD ||
      phase_ == MissionPhase::DELIVERY_DROP_HOLD ||
      phase_ == MissionPhase::DELIVERY_DESCEND_PICK ||
      phase_ == MissionPhase::DELIVERY_DESCEND_DROP) {
    // Visual servo: forward circle_center to fine_data already happens in callback;
    // here we monitor time-based transitions and align-then-descend logic.

    if (phase_ == MissionPhase::DELIVERY_DESCEND_PICK ||
        phase_ == MissionPhase::DELIVERY_DESCEND_DROP) {
      double cur_x = 0.0, cur_y = 0.0, cur_z = 0.0, cur_yaw = 0.0;
      if (!getCurrentPose(cur_x, cur_y, cur_z, cur_yaw)) {
        return;
      }

      const bool aligned = hasFreshHoverVisualData(now()) && isHoverVisuallyAligned();
      const bool height_ok = std::fabs(current_target_.z_cm - cur_z) <= height_tol_cm_;

      if (aligned && height_ok) {
        if (!delivery_align_started_) {
          delivery_align_start_time_ = now();
          delivery_align_started_ = true;
        } else if ((now() - delivery_align_start_time_).seconds() >= delivery_align_settle_sec_) {
          if (phase_ == MissionPhase::DELIVERY_DESCEND_PICK) {
            phase_ = MissionPhase::DELIVERY_PICK_HOLD;
            publishServoCommand(true);
            publishMagnetCommand(true);
            delivery_phase_start_time_ = now();
            RCLCPP_INFO(get_logger(), "Delivery: PICK_HOLD start (servo down, magnet on)");
          } else {
            phase_ = MissionPhase::DELIVERY_DROP_HOLD;
            publishServoCommand(true);
            publishMagnetCommand(false);
            delivery_phase_start_time_ = now();
            RCLCPP_INFO(get_logger(), "Delivery: DROP_HOLD start (servo down, magnet off)");
          }
        }
      } else {
        delivery_align_started_ = false;
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Delivery descend phase=%u target_z=%.1f cur_z=%.1f aligned=%s err=(%.1f,%.1f)",
        static_cast<unsigned>(phase_),
        current_target_.z_cm, cur_z, aligned ? "yes" : "no",
        hover_visual_error_x_px_, hover_visual_error_y_px_);
      return;
    }

    if (phase_ == MissionPhase::DELIVERY_PICK_HOLD) {
      const double elapsed = (now() - delivery_phase_start_time_).seconds();
      if (elapsed >= pickup_hold_sec_) {
        publishServoCommand(false);
        // Keep magnet ON while transporting.
        phase_ = MissionPhase::DELIVERY_ASCEND_PICK;
        const std::size_t pillar_idx = delivery_pickup_order_[delivery_step_];
        const Target & pillar = pillar_targets_[pillar_idx];
        current_target_ = Target{pillar.x_cm, pillar.y_cm, cruise_height_cm_, mission_yaw_deg_};
        publishVisualTakeoverState(false);
        publishOnPillar(false);
        publishHeightFilterEnabled(false);
        RCLCPP_INFO(get_logger(), "Delivery: PICK_HOLD done, ascending to cruise");
        publishTarget(current_target_);
      }
      return;
    }

    if (phase_ == MissionPhase::DELIVERY_DROP_HOLD) {
      const double elapsed = (now() - delivery_phase_start_time_).seconds();
      if (elapsed >= drop_hold_sec_) {
        publishServoCommand(false);
        publishMagnetCommand(true);  // restore magnet for next pickup
        phase_ = MissionPhase::DELIVERY_ASCEND_DROP;
        const Target & empty_target = pillar_targets_[empty_pillar_index_];
        current_target_ = Target{empty_target.x_cm, empty_target.y_cm, cruise_height_cm_, mission_yaw_deg_};
        publishVisualTakeoverState(false);
        publishOnPillar(false);
        publishHeightFilterEnabled(false);
        RCLCPP_INFO(get_logger(), "Delivery: DROP_HOLD done, ascending to cruise");
        publishTarget(current_target_);
      }
      return;
    }
  }

  if (phase_ == MissionPhase::LAND_ALIGN) {
    const double elapsed_sec = (now() - land_align_start_time_).seconds();
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Landing align: elapsed=%.1fs/%.1fs",
      elapsed_sec,
      landing_align_time_sec_);

    if (elapsed_sec >= landing_align_time_sec_) {
      phase_ = MissionPhase::LAND;
      current_target_ = Target{landing_x_cm_, landing_y_cm_, 0.0, mission_yaw_deg_};
      publishTarget(current_target_);
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
      resetCurrentPillarSamples();
      publishHeightFilterEnabled(true);
      publishTarget(current_target_);
      break;

    case MissionPhase::VISIT_PILLAR:
      startHoverMeasurement();
      break;

    case MissionPhase::GO_TO_LAND_POINT:
      phase_ = MissionPhase::LAND_ALIGN;
      land_align_start_time_ = now();
      publishVisualTakeoverState(true);
      publishHeightFilterEnabled(false);
      current_target_ = Target{landing_x_cm_, landing_y_cm_, cruise_height_cm_, mission_yaw_deg_};
      publishTarget(current_target_);
      break;

    case MissionPhase::LAND:
      completeMission("Mission finished. Landing target reached and mission_complete published.");
      break;

    case MissionPhase::DELIVERY_GOTO_PICK: {
      const std::size_t pillar_idx = delivery_pickup_order_[delivery_step_];
      const double target_z = pillarDescendTargetCm(pillar_idx, pickup_clearance_cm_);
      const Target & pillar = pillar_targets_[pillar_idx];
      phase_ = MissionPhase::DELIVERY_DESCEND_PICK;
      delivery_align_started_ = false;
      hover_has_visual_alignment_ = false;
      hover_last_visual_sample_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      publishVisualTakeoverState(true);
      publishOnPillar(true);
      publishHeightFilterEnabled(false);
      current_target_ = Target{pillar.x_cm, pillar.y_cm, target_z, mission_yaw_deg_};
      RCLCPP_INFO(
        get_logger(),
        "Delivery: arrived above pickup pillar %zu, descending to z=%.1fcm (clearance=%.1fcm)",
        pillar_idx + 1, target_z, pickup_clearance_cm_);
      publishTarget(current_target_);
      break;
    }

    case MissionPhase::DELIVERY_ASCEND_PICK: {
      // Reached cruise after pickup. Decide retry vs proceed.
      const bool still_has_circle = latest_circle_rank_ > 0;
      if (still_has_circle && pickup_retry_count_ + 1 < pickup_retry_max_) {
        ++pickup_retry_count_;
        const std::size_t pillar_idx = delivery_pickup_order_[delivery_step_];
        const Target & pillar = pillar_targets_[pillar_idx];
        const double target_z = pillarDescendTargetCm(pillar_idx, pickup_clearance_cm_);
        phase_ = MissionPhase::DELIVERY_DESCEND_PICK;
        delivery_align_started_ = false;
        hover_has_visual_alignment_ = false;
        publishVisualTakeoverState(true);
        publishOnPillar(true);
        current_target_ = Target{pillar.x_cm, pillar.y_cm, target_z, mission_yaw_deg_};
        RCLCPP_WARN(
          get_logger(),
          "Delivery: circle still visible (rank=%d), retry %d/%d",
          latest_circle_rank_, pickup_retry_count_, pickup_retry_max_);
        publishTarget(current_target_);
      } else {
        if (still_has_circle) {
          RCLCPP_WARN(
            get_logger(),
            "Delivery: pickup retries exhausted, still see circle. Proceeding to drop anyway.");
        } else {
          RCLCPP_INFO(get_logger(), "Delivery: pickup confirmed. Proceeding to drop.");
        }
        beginDeliveryDrop();
      }
      break;
    }

    case MissionPhase::DELIVERY_GOTO_DROP: {
      const double target_z = pillarDescendTargetCm(empty_pillar_index_, drop_clearance_cm_);
      const Target & empty_target = pillar_targets_[empty_pillar_index_];
      phase_ = MissionPhase::DELIVERY_DESCEND_DROP;
      delivery_align_started_ = false;
      hover_has_visual_alignment_ = false;
      hover_last_visual_sample_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      publishVisualTakeoverState(true);
      publishOnPillar(true);
      publishHeightFilterEnabled(false);
      current_target_ = Target{empty_target.x_cm, empty_target.y_cm, target_z, mission_yaw_deg_};
      RCLCPP_INFO(
        get_logger(),
        "Delivery: arrived above drop pillar %zu, descending to z=%.1fcm (clearance=%.1fcm)",
        empty_pillar_index_ + 1, target_z, drop_clearance_cm_);
      publishTarget(current_target_);
      break;
    }

    case MissionPhase::DELIVERY_ASCEND_DROP:
      RCLCPP_INFO(get_logger(), "Delivery: drop done, advancing to next rank");
      advanceDeliveryStep();
      break;

    case MissionPhase::WAIT_PILLARS:
    case MissionPhase::HOVER_AND_MEASURE:
    case MissionPhase::LAND_ALIGN:
    case MissionPhase::COMPLETE:
    case MissionPhase::DELIVERY_DESCEND_PICK:
    case MissionPhase::DELIVERY_PICK_HOLD:
    case MissionPhase::DELIVERY_DESCEND_DROP:
    case MissionPhase::DELIVERY_DROP_HOLD:
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
