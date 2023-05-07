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
#include <tf2/LinearMath/Matrix3x3.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// user
#include "trt_armor_detector/trt_detector.h"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "trt_armor_detector/pnp_solver.hpp"

namespace armor_auto_aim
{
class TRTDetectorNode : public rclcpp::Node
{
public:
	TRTDetectorNode(const rclcpp::NodeOptions& options);
	// ~TRTDetectorNode();

private:
	void createDebugPublishers();
	void destroyDebugPublishers();

	void drawResults(cv::Mat & img, const std::vector<bbox> & bboxes);
  	void publishMarkers();

	// 敌方颜色
  	int detect_color;

	// trt模型
	TRTDetector trt_detector;
	std::string model_path;
	float prob_threshold; //置信度阈值
	float nms_threshold;  //非极大抑制阈值

	// 图像订阅的传输类型
	std::string transport_;
	
	// 装甲板publisher
	auto_aim_interfaces::msg::Armors armors_msg_;
	rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  	std::unique_ptr<PnPSolver> pnp_solver_;

	// Visualization marker publisher
	visualization_msgs::msg::Marker armor_marker_;
	visualization_msgs::msg::Marker text_marker_;
	visualization_msgs::msg::MarkerArray marker_array_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
	
	// 监控参数的动态变化
	bool active_;
	std::shared_ptr<rclcpp::ParameterEventHandler> active_param_sub_;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> active_cb_handle_;

	std::shared_ptr<image_transport::Subscriber> img_sub_;

	// Camera info subscription
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

	// Camera info
	std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
	
    // Camera center
    cv::Point2f cam_center_;
	
	// Debug information publishers
	bool debug_;
	std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
	image_transport::Publisher final_img_pub_;
	int fps;

public:
	void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);
};
} // namespace armor_auto_aim

#endif // ARMOR_DETECTOR__DETECTOR_NODE_H