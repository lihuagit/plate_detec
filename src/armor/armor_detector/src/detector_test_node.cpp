/**
 * @file detector_test_node.cpp
 * @brief 测试detector.cpp
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-02-05
 * 
 */

// c++
#include <memory>
#include <string>
#include <vector>

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

// opencv
#include <opencv2/opencv.hpp>

// user
#include "armor_detector/detector.h"

namespace armor_auto_aim
{
class DetectorTestNode : public rclcpp::Node
{
public:
	DetectorTestNode(const rclcpp::NodeOptions& options)
	: Node("test_detector_node", options){
		// Image subscriptions transport type
		transport_ = "raw";

		// Detector
  		detector_ = initDetector();
		detector_->isDebug = true;
		// detector_->isDebug = false;

		if(detector_->isDebug){
			lights_data_pub_ =
				this->create_publisher<armor_interfaces::msg::DebugLights>("/debug/lights", 10);
			armors_data_pub_ =
				this->create_publisher<armor_interfaces::msg::DebugArmors>("/debug/armors", 10);
		}

		// Image callback
		img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
			this, "/image_raw", std::bind(&DetectorTestNode::imgCallback, this, std::placeholders::_1), transport_,
			rmw_qos_profile_sensor_data));
	}
	// ~DetectorTestNode();

private:
	// Image callback
	void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg){
		RCLCPP_INFO(this->get_logger(), "test_detector_node receive image!!!");
		// Convert ROS img to cv::Mat
		auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

		std::vector<Light> lights;
		std::vector<Armor> armors;

		// auto binary_img = detector_->preprocessImage(img);
		detector_->detectArmor(img, armors);
		// detector_->detectLights(img, lights);
		// detector_->matchLights(lights, armors);

		
		lights_data_pub_->publish(detector_->debug_lights);
		armors_data_pub_->publish(detector_->debug_armors);

		// detector_->detectArmor(img, armors);
		if(detector_->isDebug){
			if(!detector_->debug_preprocess_img.empty()){
				cv::imshow("pre_img", detector_->debug_preprocess_img);
			}
			cv::waitKey(1);
		}
		RCLCPP_INFO(this->get_logger(), "imgCallback over!!!");
	}

	// Image subscriptions transport type
	std::string transport_;

	// Image subscription
	std::shared_ptr<image_transport::Subscriber> img_sub_;

	// Detector
	std::unique_ptr<Detector> detector_;

	rclcpp::Publisher<armor_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
	rclcpp::Publisher<armor_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
	
	std::unique_ptr<Detector> initDetector()
	{
		rcl_interfaces::msg::ParameterDescriptor param_desc;
		param_desc.integer_range.resize(1);
		param_desc.integer_range[0].step = 1;
		param_desc.integer_range[0].from_value = 0;
		param_desc.integer_range[0].to_value = 255;
		int min_lightness = declare_parameter("min_lightness", 160, param_desc);

		param_desc.description = "0-RED, 1-BLUE";
		param_desc.integer_range[0].from_value = 0;
		param_desc.integer_range[0].to_value = 1;
		auto detect_color = declare_parameter("detect_color", BLUE, param_desc);

		Detector::LightParams l_params = {
			.min_ratio = declare_parameter("light.min_ratio", 0.1),
			.max_ratio = declare_parameter("light.max_ratio", 0.55),
			.max_angle = declare_parameter("light.max_angle", 40.0)};

		Detector::ArmorParams a_params = {
			.min_light_length_ratio = declare_parameter("armor.min_light_ratio", 0.6),
			.min_small_light_distance_ratio = declare_parameter("armor.min_small_center_distance", 0.8),
			.max_small_light_distance_ratio = declare_parameter("armor.max_small_center_distance", 2.8),
			.min_large_light_distance_ratio = declare_parameter("armor.min_large_center_distance", 3.2),
			.max_large_light_distance_ratio = declare_parameter("armor.max_large_center_distance", 5.0),
			.max_angle = declare_parameter("armor.max_angle", 35.0)};

		return std::make_unique<Detector>(min_lightness, detect_color, l_params, a_params);
	}

};
} // namespace armor_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::DetectorTestNode)