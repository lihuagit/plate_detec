/**
 * @file serial_node.h
 * @brief 串口
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-04
 * 
 */

#ifndef SERIAL__SERIAL_NODE_H_
#define SERIAL__SERIAL_NODE_H_

// c++
#include <vector>
#include <cmath>

// ros
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// user
#include "auto_aim_interfaces/msg/target.hpp"
#include <lc_serial/cJSON.h>
// #include <lc_serial/../../src/cJSON.c>
#include <lc_serial/malloc.h>
#include <lc_serial/coordsolver.h>
// #include <lc_serial/../../src/coordsolver.cpp>

class SerialDriver : public rclcpp::Node
{
public:
  explicit SerialDriver(const rclcpp::NodeOptions & options);

  ~SerialDriver() override;

private:
  void getParams();

  void receiveData();

  void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

  void reopenPort();

  std::unique_ptr<CoordSolver> coord_solver_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  

  std::thread receive_thread_;

  double shoot_speed_;
  double shoot_delay_;
  double shoot_delay_spin_;
  double z_gain;
  double y_gain;
  double pitch_gain_;

  bool is_track;
  bool is_pitch_gain;
  
  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

#endif  // SERIAL__SERIAL_NODE_H_