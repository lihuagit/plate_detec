/**
 * @file serial_old_node.h
 * @brief 老串口
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-04
 * 
 */
// c++
#include <vector>

// ros
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

// user
#include <armor_interfaces/msg/target_info.hpp>
#include <serial/cJSON.h>
#include <serial/../../src/cJSON.c>
#include <serial/malloc.h>
#include <serial/serial.h>

class SerialOldDriver : public rclcpp::Node
{
public:
  explicit SerialOldDriver(const rclcpp::NodeOptions & options);

  ~SerialOldDriver() override;

private:
  void getParams();

  void receiveData();

  void sendData(armor_interfaces::msg::TargetInfo::SharedPtr msg);

  void reopenPort();

  Serial serial(115200);

  rclcpp::Subscription<armor_interfaces::msg::TargetInfo>::SharedPtr target_sub_;

  std::thread receive_thread_;
};
