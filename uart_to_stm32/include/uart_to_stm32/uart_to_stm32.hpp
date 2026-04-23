#ifndef UART_TO_STM32__UART_TO_STM32_HPP_
#define UART_TO_STM32__UART_TO_STM32_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial_comm/serial_comm.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace uart_to_stm32
{

class UartToStm32
{
public:
  explicit UartToStm32(rclcpp::Node::SharedPtr node);
  ~UartToStm32();

  bool initialize(double update_rate, const std::string & source_frame, const std::string & target_frame);

private:
  void lookupTransform();
  void processTfTransform(const geometry_msgs::msg::TransformStamped & transform);
  void activeControllerCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void missionCompleteCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void onPillarCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void heightFilterEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void servoCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void pillarSignalTimerCallback();
  void protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data);
  void publishFilteredHeight(int16_t raw_value_cm);

  Eigen::Vector3d transformVelocity(const Eigen::Vector3d & linear, double yaw);
  void sendVelocityToSerial(const Eigen::Vector3d & transformed_velocity);
  void sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s);
  void sendMissionCompleteToSerial();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  double update_rate_;
  std::string source_frame_;
  std::string target_frame_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr active_controller_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr mission_complete_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr on_pillar_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr height_filter_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_command_sub_;
  rclcpp::TimerBase::SharedPtr pillar_signal_timer_;

  std::unique_ptr<serial_comm::SerialComm> serial_comm_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_st_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mission_step_pub_;

  double current_yaw_;
  bool yaw_valid_;
  geometry_msgs::msg::Twist current_velocity_;
  bool velocity_valid_;
  bool route_task_active_;
  bool has_st_ready_pub_;

  std::atomic<bool> on_pillar_{false};
  std::atomic<bool> height_filter_enabled_{false};

  int cruise_height_cm_;
  int height_band_cm_;

  static constexpr uint8_t VELOCITY_FRAME_ID = 0x32;
  static constexpr uint8_t TARGET_VELOCITY_FRAME_ID = 0x31;
  static constexpr uint8_t ST_READY_QUERY_ID = 0xF1;
  static constexpr uint8_t MISSION_COMPLETE_FRAME_ID = 0x66;
  static constexpr uint8_t MISSION_COMPLETE_VALUE = 0x06;
  static constexpr uint8_t PILLAR_SIGNAL_FRAME_ID = 0x22;
  static constexpr uint8_t SERVO_FRAME_ID = 0x33;
};

}  // namespace uart_to_stm32

#endif  // UART_TO_STM32__UART_TO_STM32_HPP_
