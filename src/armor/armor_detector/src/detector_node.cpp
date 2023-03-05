/**
 * @file detector_node.cpp
 * @brief
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-02-04
 *
 */

#include "armor_detector/detector_node.h"

// c++
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

// ros
#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>

// opencv
#include <opencv2/opencv.hpp>

// user
#include "armor_detector/armor.h"

using std::placeholders::_1;

namespace armor_auto_aim
{
DetectorNode::DetectorNode(const rclcpp::NodeOptions& options) : Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // Detector
  detector_ = initDetector();

  // Number classifier
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/fc.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier.threshold", 0.7);
  classifier_ = std::make_unique<NumberClassifier>(model_path, label_path, threshold);

  // Armors Publisher
  armors_pub_ = this->create_publisher<armor_interfaces::msg::Armors>("/detector/armors", rclcpp::SensorDataQoS());

  // // Visualization Marker Publisher
  // position_marker_.ns = "armors";
  // position_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  // position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  // position_marker_.color.a = 1.0;
  // position_marker_.color.r = 1.0;

  // text_marker_.ns = "classification";
  // text_marker_.action = visualization_msgs::msg::Marker::ADD;
  // text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  // text_marker_.scale.z = 0.1;
  // text_marker_.color.a = 1.0;
  // text_marker_.color.r = 1.0;
  // text_marker_.color.g = 1.0;
  // text_marker_.color.b = 1.0;
  // text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // marker_pub_ =
  //   this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  if (debug_)
  {
	createDebugPublishers();
  }

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ = debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter& p) {
	debug_ = p.as_bool();
	debug_ ? createDebugPublishers() : destroyDebugPublishers();
  });

  //   RCLCPP_INFO(this->get_logger(), "Starting camera!");

  //   cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
  //     "/camera_info", rclcpp::SensorDataQoS(),
  //     [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
  //       cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
  //       cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
  //       pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
  //       cam_info_sub_.reset();
  //     });

  RCLCPP_INFO(this->get_logger(), "Starting img!");

  // Subscriptions transport type
  transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

  img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
	  this, "/image_raw", std::bind(&DetectorNode::imageCallback, this, _1), transport_, rmw_qos_profile_sensor_data));

  RCLCPP_INFO(this->get_logger(), "over img!");

  active_ = this->declare_parameter("active", true);
  active_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  active_cb_handle_ = active_param_sub_->add_parameter_callback("active", [this](const rclcpp::Parameter& p) {
	active_ = p.as_bool();
	if (active_)
	{
	  if (img_sub_ == nullptr)
	  {
		img_sub_ = std::make_shared<image_transport::Subscriber>(
			image_transport::create_subscription(this, "/image_raw", std::bind(&DetectorNode::imageCallback, this, _1),
												 transport_, rmw_qos_profile_sensor_data));
	  }
	}
	else if (img_sub_ != nullptr)
	{
	  img_sub_.reset();
	}
  });

  RCLCPP_INFO(this->get_logger(), "over!");
}

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  RCLCPP_INFO(this->get_logger(), "Receive img!");
  auto armors = detectArmors(img_msg);
  armors_msg_.header = img_msg->header;
  // armors_msg_.header = position_marker_.header = text_marker_.header = img_msg->header;
  armors_msg_.armors.clear();
  // marker_array_.markers.clear();
  // position_marker_.points.clear();
  // text_marker_.id = 0;

  armor_interfaces::msg::Armor armor_msg;
  for (const auto& armor : armors)
  {
	std::vector<geometry_msgs::msg::Point> positions(4);
	positions[0].x = armor.left_light.bottom.x;
	positions[0].y = armor.left_light.bottom.y;
	positions[0].z = 0;
	positions[1].x = armor.left_light.top.x;
	positions[1].y = armor.left_light.top.y;
	positions[1].z = 0;
	positions[2].x = armor.right_light.top.x;
	positions[2].y = armor.right_light.top.y;
	positions[2].z = 0;
	positions[3].x = armor.right_light.bottom.x;
	positions[3].y = armor.right_light.bottom.y;
	positions[3].z = 0;

	// 使positions赋值给armor_msg.positions
	armor_msg.positions = positions;

	armor_msg.number = armor.number;

	armors_msg_.armors.emplace_back(armor_msg);

	// Publishing detected armors
	armors_pub_->publish(armors_msg_);
  }
}

std::unique_ptr<Detector> DetectorNode::initDetector()
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

  Detector::LightParams l_params = { .min_ratio = declare_parameter("light.min_ratio", 0.1),
									 .max_ratio = declare_parameter("light.max_ratio", 0.55),
									 .max_angle = declare_parameter("light.max_angle", 40.0) };

  Detector::ArmorParams a_params = {
	.min_light_length_ratio = declare_parameter("armor.min_light_ratio", 0.6),
	.min_small_light_distance_ratio = declare_parameter("armor.min_small_center_distance", 0.8),
	.max_small_light_distance_ratio = declare_parameter("armor.max_small_center_distance", 2.8),
	.min_large_light_distance_ratio = declare_parameter("armor.min_large_center_distance", 3.2),
	.max_large_light_distance_ratio = declare_parameter("armor.max_large_center_distance", 5.0),
	.max_angle = declare_parameter("armor.max_angle", 35.0)
  };

  bool is_debug = declare_parameter("is_debug", true);

  return std::make_unique<Detector>(min_lightness, detect_color, l_params, a_params, is_debug);
}

std::vector<Armor> DetectorNode::detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  auto start_time = this->now();
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
  RCLCPP_INFO(this->get_logger(), "detectArmors img!");

  // Detect armors
  detector_->min_lightness = get_parameter("min_lightness").as_int();
  detector_->detect_color = get_parameter("detect_color").as_int();

  std::vector<Light> lights;
  std::vector<Armor> armors;

  // detectLights(src, lights);

  // auto binary_img = detector_->preprocessImage(img);
  detector_->detectLights(img, lights);
  detector_->matchLights(lights, armors);

  // Extract numbers
  if (!armors.empty())
  {
    classifier_->extractNumbers(img, armors);
    classifier_->threshold = get_parameter("classifier.threshold").as_double();
    classifier_->doClassify(armors);
  }

  // Publish debug info
  if (debug_)
  {
	auto final_time = this->now();
	auto latency = (final_time - start_time).seconds() * 1000;
	RCLCPP_INFO_STREAM(this->get_logger(), "detectArmors used: " << latency << "ms");
	cv::putText(img, "Latency: " + std::to_string(latency) + "ms", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
				cv::Scalar(0, 255, 0), 2);

	// binary_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", binary_img).toImageMsg());

	std::sort(detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
			  [](const auto& l1, const auto& l2) { return l1.center_x < l2.center_x; });
	std::sort(detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
			  [](const auto& a1, const auto& a2) { return a1.center_x < a2.center_x; });

	lights_data_pub_->publish(detector_->debug_lights);
	armors_data_pub_->publish(detector_->debug_armors);

	if (!armors.empty())
	{
	  // Combine all number images to one
	  std::vector<cv::Mat> number_imgs;
	  number_imgs.reserve(armors.size());
	  for (auto& armor : armors)
	  {
		cv::resize(armor.number_img, armor.number_img, cv::Size(20, 28));
		number_imgs.emplace_back(armor.number_img);
	  }
	  cv::Mat all_num_img;
	  cv::vconcat(number_imgs, all_num_img);

	  number_pub_->publish(*cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
	}

	drawResults(img, lights, armors);
	final_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
	if (!detector_->preprocess_img.empty())
		preprocess_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", detector_->preprocess_img).toImageMsg());
  }

  return armors;
}

void DetectorNode::drawResults(cv::Mat& img, const std::vector<Light>& lights, const std::vector<Armor>& armors)
{
  // Draw Lights
  for (const auto& light : lights)
  {
	auto color = light.color == RED ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
	cv::ellipse(img, light, color, 2);
  }

  // Draw armors
  for (const auto& armor : armors)
  {
	cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
	cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  for (const auto& armor : armors)
  {
	cv::putText(img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
				cv::Scalar(0, 255, 255), 2);
  }

  // Draw camera center
  cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
}

void DetectorNode::createDebugPublishers()
{
  lights_data_pub_ = this->create_publisher<armor_interfaces::msg::DebugLights>("/debug/lights", 10);
  armors_data_pub_ = this->create_publisher<armor_interfaces::msg::DebugArmors>("/debug/armors", 10);
  number_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/number", 10);

  preprocess_img_pub_ = image_transport::create_publisher(this, "/binary_img");
  final_img_pub_ = image_transport::create_publisher(this, "/final_img");
}

void DetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();
  number_pub_.reset();

  preprocess_img_pub_.shutdown();
  final_img_pub_.shutdown();
}

// void DetectorNode::publishMarkers()
// {
//   using Marker = visualization_msgs::msg::Marker;
//   position_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
//   marker_array_.markers.emplace_back(position_marker_);
//   marker_pub_->publish(marker_array_);
// }
}  // namespace armor_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::DetectorNode)