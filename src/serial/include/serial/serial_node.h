/**
 * @file serial_node.h
 * @brief 串口
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
#include <serial_driver/serial_driver.hpp>

// user
#include <armor_interfaces/msg/target_info.hpp>
#include <serial/cJSON.h>
#include <serial/../../src/cJSON.c>
#include <serial/malloc.h>

#define CMD_NAME_LENGTH 100
#define MAX_CMD_NUM 10

class SerialDriver : public rclcpp::Node
{
public:
  explicit SerialDriver(const rclcpp::NodeOptions & options);

  ~SerialDriver() override;

private:
  void getParams();

  void receiveData();

  void sendData(armor_interfaces::msg::TargetInfo::SharedPtr msg);

  void reopenPort();

  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  rclcpp::Subscription<armor_interfaces::msg::TargetInfo>::SharedPtr target_sub_;

  std::thread receive_thread_;
};
