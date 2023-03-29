/**
 * @file serial_node.cpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-04
 * 
 */

#include "serial/serial_old_node.h"


SerialOldDriver::SerialOldDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options)
{
  RCLCPP_INFO(get_logger(), "Start SerialOldDriver!");

  getParams();

  serial_ = std::make_unique<Serial>(115200);

  // Create Subscription
  target_sub_ = this->create_subscription<armor_interfaces::msg::TargetInfo>(
    "/processor/target", rclcpp::SensorDataQoS(),
    std::bind(&SerialOldDriver::sendData, this, std::placeholders::_1));
}

SerialOldDriver::~SerialOldDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

/**
 * @brief 发送数据
 * @param msg 
 */
void SerialOldDriver::sendData(const armor_interfaces::msg::TargetInfo::SharedPtr msg)
{
  try {
    // std::vector<uint8_t> data;
    //TODO:自定义发送格式
    uint8_t buff[10];
    
    buff[0] = 's';
    // printf("x:%f\n",x);
    float test = msg->euler.x;
    memcpy(buff + 1, &test, 4);
    test = msg->euler.y;
    memcpy(buff + 5, &test, 4);
    // printf("data: %f\n", *(float*)(buff + 1));
    buff[9] = 'e';

    serial_->WriteData(buff, sizeof(buff));

    RCLCPP_INFO(get_logger(), "SerialOldDriver sending data: %c , %f, %f, %c", buff[0], *(float*)(buff + 1), *(float*)(buff + 5), buff[9]);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
  }
}

/**
 * @brief 不断接收电控数据
 */
void SerialOldDriver::receiveData()
{
  char buffer[40];
  while (rclcpp::ok()) {
    try {
      memset(buffer, 0, sizeof(buffer));
      serial_->ReadData((uint8_t *) buffer, 15);
      // TODO:收到电控数据
      RCLCPP_INFO(get_logger(), "SerialOldDriver receiving data: %s", buffer);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error while receiving data: %s", ex.what());
    }
  }
}

void SerialOldDriver::getParams()
{
  // try {
  //   std::string device_name_ = declare_parameter<std::string>("device_name", "");
  // } catch (rclcpp::ParameterTypeException & ex) {
  //   RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
  //   throw ex;
  // }

  // try {
  //   int baud_rate = declare_parameter<int>("baud_rate", 0);
  // } catch (rclcpp::ParameterTypeException & ex) {
  //   RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
  //   throw ex;
  // }

  // try {
  //   const auto fc_string = declare_parameter<std::string>("flow_control", "none");
  // } catch (rclcpp::ParameterTypeException & ex) {
  //   RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
  //   throw ex;
  // }

  // try {
  //   const auto pt_string = declare_parameter<std::string>("parity", "none");
  // } catch (rclcpp::ParameterTypeException & ex) {
  //   RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
  //   throw ex;
  // }

  // try {
  //   const auto sb_string = declare_parameter<std::string>("stop_bits", "1");
  // } catch (rclcpp::ParameterTypeException & ex) {
  //   RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
  //   throw ex;
  // }
}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(SerialOldDriver)