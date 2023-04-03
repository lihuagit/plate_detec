/**
 * @file armor_tracker_node.h
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-02-18
 * 
 */

#ifndef ARMOR_TRACKER__ARMOR_TRACKER_NODE_H
#define ARMOR_TRACKER__ARMOR_TRACKER_NODE_H

// c++
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

// user
#include "armor_tracker/tracker.h"
// #include "armor_tracker/tracker_kalman.h"
#include "armor_tracker/general.h"
#include "armor_tracker/coordsolver.h"
#include "armor_interfaces/msg/armors.hpp"
#include "armor_interfaces/msg/target_info.hpp"
#include "armor_interfaces/msg/target.hpp"

namespace armor_auto_aim
{
enum SpinHeading {UNKNOWN, CLOCKWISE, COUNTER_CLOCKWISE};
enum TargetType {SMALL, BIG, BUFF};

/**
 * @brief 记录装甲板追踪器信息
 */
struct TrackerParams{
    std::vector<ArmorTracker> armor_tracker;
    SpinHeading spin_heading;   //反小陀螺，记录该车小陀螺状态
    double spin_score;  //反小陀螺，记录各装甲板小陀螺可能性分数，大于0为逆时针旋转，小于0为顺时针旋转
};

class ArmorTrackerNode : public rclcpp::Node
{
using tf2_filter = tf2_ros::MessageFilter<armor_interfaces::msg::Armors>;
public:
    ArmorTrackerNode(const rclcpp::NodeOptions& options);
    // ~ArmorTrackerNode();

    CoordSolver coord_solver;

private:

    int anti_spin_judge_high_thres = 2e4;//大于该阈值认为该车已开启陀螺
    int anti_spin_judge_low_thres = 2e3;//小于该阈值认为该车已关闭陀螺
    double hero_danger_zone;

    double shoot_v; //射击速度

    unordered_map<std::string, TrackerParams> trackers_map;    //预测器Map
    std::map<string,int> new_armors_cnt_map;    //装甲板计数map，记录新增装甲板数

    std::unique_ptr<ArmorTracker> tracker_ptr_; 

    // Camera center
    cv::Point2f cam_center_;

    // Detected armors publisher
    // rclcpp::Subscription<armor_interfaces::msg::Armors>::SharedPtr armors_sub_;
    
    // Subscriber with tf2 message_filter
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<armor_interfaces::msg::Armors> armors_sub_;
    std::shared_ptr<tf2_filter> tf2_filter_;
    
    // Visualization marker publisher
    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker predict_position_marker_;
    visualization_msgs::msg::Marker velocity_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;


    // 发布击打目标信息
	rclcpp::Publisher<armor_interfaces::msg::TargetInfo>::SharedPtr target_info_pub_;

    // armors callback
    void armorsCallback(armor_interfaces::msg::Armors::ConstSharedPtr armors_msg);
    
    // 更新反陀螺参数
    bool updateSpinScore();
    std::string chooseTargetID(vector<Armor> &armors);
    Armor chooseTargetArmor(vector<Armor> armors);
    void createTrackers();
    void publishMarkers(const Armor & target_armor, const std_msgs::msg::Header& header);

    // Debug
    bool debug_;
	std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
	image_transport::Publisher tracker_img_pub_;
};
} // namespace armor_auto_aim

#endif // ARMOR_TRACKER__ARMOR_TRACKER_NODE_H