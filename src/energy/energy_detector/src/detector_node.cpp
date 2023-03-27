/**
 * @file detector_node.cpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-27
 * 
 */

#include "energy_detector/detector_node.h"

namespace energy_auto_aim
{
DetectorNode::DetectorNode(const rclcpp::NodeOptions& options)
    : Node("energy_detector", options)
{
	RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");
	
    // 获取参数
    transport_ = this->declare_parameter<std::string>("transport", "raw");

    // 订阅图像
    img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/image_raw", 
				std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1), transport_, rmw_qos_profile_sensor_data));

	// 初始化能量机关识别器
	std::string model_path = ament_index_cpp::get_package_share_directory("energy_detector") + "/model/buff.xml";
	if (!detector_.initModel(model_path))
		RCLCPP_ERROR(this->get_logger(), "Detector init model failed!");

    // active_ 监控参数的动态变化
    active_ = this->declare_parameter<bool>("active", true);
    active_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
	active_cb_handle_ = active_param_sub_->add_parameter_callback("active", [this](const rclcpp::Parameter& p) {
		active_ = p.as_bool();
		if (active_)
		{
			if (img_sub_ == nullptr)
			{
				img_sub_ = std::make_shared<image_transport::Subscriber>(
					image_transport::create_subscription(this, "/image_raw", std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1),
														transport_, rmw_qos_profile_sensor_data));
			}
		}
		else if (img_sub_ != nullptr)
		{
			img_sub_.reset();
		}
	});

	// debug 监控参数的动态变化
	debug_ = this->declare_parameter<bool>("debug", true);
	if (debug_)
		createDebugPublishers();
	debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
	debug_cb_handle_ = debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter& p) {
		debug_ = p.as_bool();
		debug_ ? createDebugPublishers() : destroyDebugPublishers();
	});
}

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)

{
	// RCLCPP_INFO(this->get_logger(), "receive image!");
	// 1. 将ROS图像消息转换为OpenCV图像
	cv::Mat img;
	try
	{
		img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
	}
	catch (cv_bridge::Exception & e)
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}

	// 2. 识别能量机关
	std::vector<BuffObject> results;
	detector_.detect(img, results);

	// 3. 绘制结果
	cv::Mat final_img = img.clone();
	detector_.drawResult(final_img, results);

	if(debug_){
		final_img_pub_->publish(cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", final_img).toImageMsg());
	}
}

void DetectorNode::createDebugPublishers(){
	final_img_pub_ = std::make_shared<image_transport::Publisher>(
					image_transport::create_publisher(this, "/final_img")); 
}

void DetectorNode::destroyDebugPublishers(){
	final_img_pub_.reset();
}

} // namespace energy_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(energy_auto_aim::DetectorNode)