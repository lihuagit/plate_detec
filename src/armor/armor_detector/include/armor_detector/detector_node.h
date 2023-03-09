/**
 * @file detector_node.h
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-02-04
 * 
 */
#ifndef ARMOR_DETECTOR__DETECTOR_NODE_H
#define ARMOR_DETECTOR__DETECTOR_NODE_H

// c++
#include <memory>
#include <string>
#include <vector>

// ros
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// user
#include "armor_detector/detector.h"
#include "armor_detector/number_classifier.h"
#include "armor_interfaces/msg/armors.hpp"

namespace armor_auto_aim
{
class DetectorNode : public rclcpp::Node
{
public:
	DetectorNode(const rclcpp::NodeOptions& options);
	// ~DetectorNode();

private:
  	std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

	// Camera info subscription
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

	// Camera info
	std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

	// Camera center
	cv::Point2f cam_center_;

	// Image subscriptions transport type
	std::string transport_;
	
	// Detected armors publisher
	armor_interfaces::msg::Armors armors_msg_;
	rclcpp::Publisher<armor_interfaces::msg::Armors>::SharedPtr armors_pub_;
	
	bool active_;
	std::shared_ptr<rclcpp::ParameterEventHandler> active_param_sub_;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> active_cb_handle_;
public:
	void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

	std::shared_ptr<image_transport::Subscriber> img_sub_;

  	std::unique_ptr<Detector> initDetector();

	void createDebugPublishers();
	void destroyDebugPublishers();

	void drawResults(
		cv::Mat & img, const std::vector<Light> & lights, const std::vector<Armor> & armors);

	// Armor Detector
	std::unique_ptr<Detector> detector_;
	
	// Number Classifier
	std::unique_ptr<NumberClassifier> classifier_;
	
	// Debug information publishers
	bool debug_;
	std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
	rclcpp::Publisher<armor_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
	rclcpp::Publisher<armor_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr number_pub_;
	image_transport::Publisher preprocess_img_pub_;
	image_transport::Publisher final_img_pub_;
};
} // namespace armor_auto_aim

#endif // ARMOR_DETECTOR__DETECTOR_NODE_H