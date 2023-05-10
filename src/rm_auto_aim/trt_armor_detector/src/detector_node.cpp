/**
 * @file detector_node.cpp
 * @brief
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-02-04
 *
 */

#include "trt_armor_detector/detector_node.h"

using std::placeholders::_1;

namespace armor_auto_aim
{
TRTDetectorNode::TRTDetectorNode(const rclcpp::NodeOptions& options) : Node("trt_armor_detector", options)
{
	RCLCPP_INFO(this->get_logger(), "Starting TRTDetectorNode!");

	// trt_model
	auto pkg_path = ament_index_cpp::get_package_share_directory("trt_armor_detector");
	model_path = this->declare_parameter("model_path", "armor640_20230401.trt");
	model_path = pkg_path + "/model/" + model_path;
	trt_detector.loadTrtModel(model_path.data());

	detect_color = this->declare_parameter("detect_color", 1);
	prob_threshold = this->declare_parameter("prob_threshold", 0.500);
	nms_threshold = this->declare_parameter("nms_threshold", 0.500);

	// Armors Publisher
	armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>("/detector/armors", rclcpp::SensorDataQoS());

	// Visualization Marker Publisher
	// See http://wiki.ros.org/rviz/DisplayTypes/Marker
	armor_marker_.ns = "armors";
	armor_marker_.action = visualization_msgs::msg::Marker::ADD;
	armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
	armor_marker_.scale.x = 0.03;
	armor_marker_.scale.y = 0.15;
	armor_marker_.scale.z = 0.12;
	armor_marker_.color.a = 1.0;
	armor_marker_.color.r = 1.0;
	armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

	text_marker_.ns = "classification";
	text_marker_.action = visualization_msgs::msg::Marker::ADD;
	text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
	text_marker_.scale.z = 0.1;
	text_marker_.color.a = 1.0;
	text_marker_.color.r = 1.0;
	text_marker_.color.g = 1.0;
	text_marker_.color.b = 1.0;
	text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

	marker_pub_ =
		this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

	// Debug param change moniter
	debug_ = this->declare_parameter("debug", true);
	debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
	debug_cb_handle_ = debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter& p) {
		debug_ = p.as_bool();
		debug_ ? createDebugPublishers() : destroyDebugPublishers();
	});

	// Debug Publishers
	debug_ = this->get_parameter("debug").as_bool();
	if (debug_)
	{
		createDebugPublishers();
	}

	cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		"/camera_info", rclcpp::SensorDataQoS(),
		[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
			cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
			cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
		  	pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
			cam_info_sub_.reset();
		});

	// Subscriptions transport type
	transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

	img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
		this, "/image_raw", std::bind(&TRTDetectorNode::imageCallback, this, _1), transport_, rmw_qos_profile_sensor_data));


	active_ = this->declare_parameter("active", true);
	active_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
	active_cb_handle_ = active_param_sub_->add_parameter_callback("active", [this](const rclcpp::Parameter& p) {
		active_ = p.as_bool();
		if (active_)
		{
			if (img_sub_ == nullptr)
			{
				img_sub_ = std::make_shared<image_transport::Subscriber>(
					image_transport::create_subscription(this, "/image_raw", std::bind(&TRTDetectorNode::imageCallback, this, _1),
														transport_, rmw_qos_profile_sensor_data));
			}
		}
		else if (img_sub_ != nullptr)
		{
			img_sub_.reset();
		}
	});

	fps = 0;
	RCLCPP_INFO(this->get_logger(), "over img!");
}

/**
 * @brief 图像回调函数，检测装甲板
 * @param img_msg 接受到的图像消息
 */
void TRTDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
	if(debug_){
		static rclcpp::Time last_time = this->now();
		static int fps_tmp = 0;
		auto start_time = this->now();

		if((start_time - last_time).seconds() < 1){
			fps_tmp++;
		}
		else{
			fps = fps_tmp;
			RCLCPP_INFO(rclcpp::get_logger("trt_armor_detector"), "TRTDetector FPS: %d", fps);
			fps_tmp = 0;
			last_time = start_time;
		}
	}

	auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
	// return ;

	/// 检测装甲板
    std::vector<bbox> bboxes;
	prob_threshold = this->get_parameter("prob_threshold").as_double();
	nms_threshold= this->get_parameter("nms_threshold").as_double();

	// 判断是否喂BRG格式，若为RGB，则转BRG
	cv::Mat detector_img;
	cv::cvtColor(img, detector_img, cv::COLOR_RGB2BGR);
	trt_detector.detect(detector_img, bboxes, prob_threshold, nms_threshold);
	
	if (pnp_solver_ != nullptr) {
		armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
		armors_msg_.armors.clear();
		marker_array_.markers.clear();
		armor_marker_.id = 0;
		text_marker_.id = 0;

		auto_aim_interfaces::msg::Armor armor_msg;
		for (const auto & armor : bboxes) {
			if(armor.color != detect_color) continue;
			cv::Mat rvec, tvec;
			bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
			if (success) {
				armor_msg.number = armor.number;
				// Fill armor_msg with pose
				armor_msg.pose.position.x = tvec.at<double>(0);
				armor_msg.pose.position.y = tvec.at<double>(1);
				armor_msg.pose.position.z = tvec.at<double>(2);
				// rvec to 3x3 rotation matrix
				cv::Mat rotation_matrix;
				cv::Rodrigues(rvec, rotation_matrix);
				// rotation matrix to quaternion
				tf2::Matrix3x3 tf2_rotation_matrix(
				rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
				rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
				rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
				rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
				rotation_matrix.at<double>(2, 2));
				tf2::Quaternion tf2_quaternion;
				tf2_rotation_matrix.getRotation(tf2_quaternion);
				armor_msg.pose.orientation.x = tf2_quaternion.x();
				armor_msg.pose.orientation.y = tf2_quaternion.y();
				armor_msg.pose.orientation.z = tf2_quaternion.z();
				armor_msg.pose.orientation.w = tf2_quaternion.w();
				// Fill the distance to image center
				cv::Point2f center;
				center.x += armor.key_points[0] + armor.key_points[2] + armor.key_points[4] + armor.key_points[6];
				center.y += armor.key_points[1] + armor.key_points[3] + armor.key_points[5] + armor.key_points[7];
				center /= 4;
				armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(center);
				// Fill the markers
				armor_marker_.id++;
				armor_marker_.pose = armor_msg.pose;
				text_marker_.id++;
				text_marker_.pose.position = armor_msg.pose.position;
				text_marker_.pose.position.y -= 0.1;
				text_marker_.text = std::to_string(armor.label) + " : " + std::to_string(armor.score);
				armors_msg_.armors.emplace_back(armor_msg);
				marker_array_.markers.emplace_back(armor_marker_);
				marker_array_.markers.emplace_back(text_marker_);
			} else {
				RCLCPP_WARN(this->get_logger(), "PnP failed!");
			}
		}

		// Publishing detected armors
		armors_pub_->publish(armors_msg_);

		// Publishing marker
		publishMarkers();
	}

	/// debug
	if(debug_){
		drawResults(img, bboxes);
		// cv::imshow("img", img);
		// cv::waitKey(1);
		final_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
	}
	// return ;
}

/**
 * @brief 绘制识别的装甲板结果
 * @param img 待绘制的图像
 * @param lights 需绘制的灯条数组
 * @param bboxes 需绘制的装甲板数组
 */
void TRTDetectorNode::drawResults(cv::Mat& img, const std::vector<bbox>& bboxes)
{
	for(auto box : bboxes){
		// Draw rect
		cv::rectangle(img,cv::Point(box.x1,box.y1),cv::Point(box.x2,box.y2),cv::Scalar(0,255,0), 2);
		// cv::circle(img, cv::Point(box.x1, box.y1), 2, cv::Scalar(255, 0, 0), 1, 1);
		// cv::circle(img, cv::Point(box.x2, box.y2), 2, cv::Scalar(255, 0, 0), 1, 1);

		// Draw key points
		cv::circle(img, cv::Point(box.key_points[0], box.key_points[1]), 2, cv::Scalar(255, 0, 0), 1, 2);
		cv::circle(img, cv::Point(box.key_points[2], box.key_points[3]), 2, cv::Scalar(255, 0, 0), 3, 2);
		cv::circle(img, cv::Point(box.key_points[4], box.key_points[5]), 2, cv::Scalar(255, 0, 0), 5, 2);
		cv::circle(img, cv::Point(box.key_points[6], box.key_points[7]), 2, cv::Scalar(255, 0, 0), 7, 2);

		// Draw label、score
		std::string res_str = std::to_string(box.label) + " : " + std::to_string(box.score);
		cv::putText(img, res_str, cv::Point(box.x1,box.y1), cv::FONT_HERSHEY_SIMPLEX, 0.8,
					cv::Scalar(0, 255, 255), 2);
	}

	// Draw camera center
	cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);

	// Draw fps
	cv::putText(img, "fps: " + std::to_string(fps) , cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
				cv::Scalar(0, 255, 0), 2);
}

void TRTDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

/**
 * @brief 开启调试模式
 */
void TRTDetectorNode::createDebugPublishers()
{
	final_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

/**
 * @brief 关闭调试模式
 */
void TRTDetectorNode::destroyDebugPublishers()
{
	final_img_pub_.shutdown();
}

}  // namespace armor_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::TRTDetectorNode)