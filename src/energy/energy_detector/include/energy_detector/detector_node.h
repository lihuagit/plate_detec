/**
 * @file detector_node.h
 * @brief 能量机关识别节点
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-27
 * 
 */

#ifndef ENERGY_DETECTOR__DETECTOR_NODE_H
#define ENERGY_DETECTOR__DETECTOR_NODE_H

// c++
#include <memory>
#include <string>
#include <vector>

// ros
#include <rclcpp/rclcpp.hpp> 
#include <rclcpp/publisher.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// other
#include "energy_detector/detector.h"
#include "armor_interfaces/msg/armors.hpp"

namespace energy_auto_aim
{
class DetectorNode : public rclcpp::Node
{
public:
	DetectorNode(const rclcpp::NodeOptions& options);

private:
    // 能量机关识别器
    Detector detector_;

    // 图像订阅的传输类型
    std::string transport_;

    // 图像订阅
	std::shared_ptr<image_transport::Subscriber> img_sub_;

    // TODO: 使用armor下的msg，待抽象出armor和energy共用的msg
    // 能量机关publisher
	rclcpp::Publisher<armor_interfaces::msg::Armors>::SharedPtr armors_pub_;
    
    // 监控参数的动态变化
    bool active_;
    std::shared_ptr<rclcpp::ParameterEventHandler> active_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> active_cb_handle_;

    // debug
    bool debug_;
	std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;

    // debug publisher
    std::shared_ptr<image_transport::Publisher> final_img_pub_;

    void createDebugPublishers();
    void destroyDebugPublishers();

public:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);
};

} // namespace energy_auto_aim

#endif // ENERGY_DETECTOR__DETECTOR_NODE_H