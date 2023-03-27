/**
 * @file video_pub.cpp
 * @brief opencv读取视频，发布图像 rclcpp
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-01-18
 *
 */
// c++
#include <chrono>
#include <functional>

// ros
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// opencv
#include <opencv2/opencv.hpp>

namespace video_pub
{
class VideoPub : public rclcpp::Node
{
public:
	VideoPub(const rclcpp::NodeOptions& options) : Node("video_pub", options)
	{
		RCLCPP_INFO(this->get_logger(), "Starting VideoPubNode!");
		video_path_ = this->declare_parameter("video_path", "/video/red_energy.mp4");
		src_img_pub_ = image_transport::create_publisher(this, "image_raw");
		auto pkg_path = ament_index_cpp::get_package_share_directory("video_pub");
		cap_ = cv::VideoCapture(pkg_path + video_path_);
		timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&VideoPub::timer_callback, this));
	}

private:
	/**
	 * @brief 定时器回调函数
	 */
	void timer_callback()
	{
		cv::Mat frame;
		cap_ >> frame;
		if (frame.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "frame is empty");
			cap_.release();
			auto pkg_path = ament_index_cpp::get_package_share_directory("video_pub");
			cap_ = cv::VideoCapture(pkg_path + video_path_);
			cap_ >> frame;
			// return;
		}
		cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
		auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", frame).toImageMsg();
		msg->header.stamp = this->now();
		src_img_pub_.publish(*msg);
		RCLCPP_INFO(this->get_logger(), "aleady publish a frame");
	}
	image_transport::Publisher src_img_pub_;
	std::string video_path_;
	rclcpp::TimerBase::SharedPtr timer_;
	// opencv打开视频
	cv::VideoCapture cap_;
};
}  // namespace video_pub

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(video_pub::VideoPub)