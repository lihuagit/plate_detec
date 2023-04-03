/**
 * @file armor_tracker_node.cpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-02-19
 * 
 */

#include <armor_tracker/tracker_node.h>

namespace armor_auto_aim
{
// DetectorNode::DetectorNode(const rclcpp::NodeOptions& options) : Node("armor_detector", options)
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions& options)
    : Node("armor_tracker_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting armor_tracker_node!");

    // 弹速
    shoot_v = this->declare_parameter("shoot_v", 15.0);

    // 位姿解算器
    auto pkg_path = ament_index_cpp::get_package_share_directory("armor_tracker");
    coord_solver.loadParam(pkg_path+"/params/coord_param.yaml", "KE0200110076");
    coord_solver.bullet_speed = shoot_v;

    // tracker
    createTrackers();

    target_info_pub_ = this->create_publisher<armor_interfaces::msg::TargetInfo>("/processor/target", rclcpp::SensorDataQoS());
    
    // armors_sub_ = this->create_subscription<armor_interfaces::msg::Armors>(
    //     "/detector/armors", rclcpp::SensorDataQoS(),std::bind(&ArmorTrackerNode::armorsCallback, this, std::placeholders::_1));

    
    // Subscriber with tf2 message_filter
    // tf2 relevant
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // subscriber and filter
    armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
    target_frame_ = this->declare_parameter("target_frame", "shooter_link");
    tf2_filter_ = std::make_shared<tf2_filter>(
        armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
        this->get_node_clock_interface(), std::chrono::duration<int>(1));
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

    // 是否调试
	debug_ = this->declare_parameter("debug", false);

    if(debug_) {
        tracker_img_pub_ = image_transport::create_publisher(this, "/tracker_img");
    }

	debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
	debug_cb_handle_ = debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter& p) {
		debug_ = p.as_bool();
        if(debug_){
	        tracker_img_pub_ = image_transport::create_publisher(this, "/tracker_img");
        }
        else {
            tracker_img_pub_.shutdown();
        }
	});

    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;

    predict_position_marker_.ns = "predict_position";
    predict_position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    predict_position_marker_.scale.x = predict_position_marker_.scale.y = predict_position_marker_.scale.z = 0.1;
    predict_position_marker_.color.a = 1.0;
    predict_position_marker_.color.r = 1.0;

    velocity_marker_.type = visualization_msgs::msg::Marker::ARROW;
    velocity_marker_.ns = "velocity";
    velocity_marker_.scale.x = 0.03;
    velocity_marker_.scale.y = 0.05;
    velocity_marker_.color.a = 1.0;
    velocity_marker_.color.b = 1.0;
    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/processor/marker", 10);

}

void ArmorTrackerNode::armorsCallback(armor_interfaces::msg::Armors::ConstSharedPtr armors_msg){
    rclcpp::Time time = armors_msg->header.stamp;

    // 现在的时间，单位s
    double time_now = time.seconds();

    cam_center_ = cv::Point2f(coord_solver.intrinsic_cpy.at<float>(0, 2), coord_solver.intrinsic_cpy.at<float>(1, 2));

    std::vector<Armor> armors;
    for(const auto & armor_msg : armors_msg->armors){
        Armor armor;
        armor.id = armor_msg.number;
        for(auto & point : armor_msg.positions){
            // armor.positions2d.push_back(cv::Point2f(point.x, -1*point.y) - cam_center_);
            armor.positions2d.push_back(cv::Point2f(point.x, point.y));
        }
        armor.area = calcTetragonArea(armor.positions2d);
        
        armor.key = armor.id;
        armor.time_stamp = time_now;
        cv::Point2f position_sum;
        for(auto & point : armor.positions2d)
            position_sum += point;
        armor.center2d = position_sum / 4.f;
        armors.push_back(armor);
    }
    
    #ifdef USING_IMU
        // TODO: 从IMU获取姿态
    #else
        Eigen::Matrix3d rmat_imu = Eigen::Matrix3d::Identity();
    #endif //USING_IMU

    // 位姿解算
    for(auto & armor : armors){
        PnPInfo pnp_result = coord_solver.pnp(armor.positions2d, rmat_imu, SOLVEPNP_IPPE);
        armor.center3d_world = pnp_result.armor_world;
        armor.center3d_cam = pnp_result.armor_cam;
        armor.euler = pnp_result.euler;
        
        geometry_msgs::msg::PointStamped ps;
        ps.header = armors_msg->header;
        ps.point.x = armor.center3d_cam[0];
        ps.point.y = armor.center3d_cam[1];
        ps.point.z = armor.center3d_cam[2];
        
        try {
            geometry_msgs::msg::Point pt = tf2_buffer_->transform(ps, target_frame_).point;
            armor.center3d_world[0] = pt.x;
            armor.center3d_world[1] = pt.y;
            armor.center3d_world[2] = pt.z;
        } catch (const tf2::ExtrapolationException & ex) {
            RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
            return ;
        }
    }

    // Tracker
    tracker_ptr_->update(armors, time_now);

    bool is_tracking = false;
    is_tracking = tracker_ptr_->predictTargetArmor(shoot_v);
    if(debug_) RCLCPP_INFO(this->get_logger(), "is_tracking: %d", is_tracking);

    Armor target_armor;

    if(is_tracking)
        target_armor = tracker_ptr_->pre_armor;
    else
        return ;
        
    auto pitch_offset = coord_solver.dynamicCalcPitchOffset(target_armor.center3d_predict);
    // auto pitch_offset = geiPitch(target_armor.center3d_predict);
    target_armor.angle = coord_solver.calcYawPitch(target_armor.center3d_predict);
    target_armor.angle.y() += pitch_offset;

    if(debug_){
        RCLCPP_INFO(this->get_logger(), "center3d_world.x: %f, center3d_world.y: %f, center3d_world.z: %f", target_armor.center3d_world.x(), target_armor.center3d_world.y(), target_armor.center3d_world.z()); 
        RCLCPP_INFO(this->get_logger(), "center3d_predict.x: %f, center3d_predict.y: %f, center3d_predict.z: %f", target_armor.center3d_predict.x(), target_armor.center3d_predict.y(), target_armor.center3d_predict.z());
        RCLCPP_INFO(this->get_logger(), "pitch_offset: %f", pitch_offset);
        RCLCPP_INFO(this->get_logger(), "angle.x: %f, angle.y: %f", target_armor.angle.x(), target_armor.angle.y());
        
        cv::Mat show_mat(1280,1024,CV_8UC3, cv::Scalar(255,255,255));
		cv::cvtColor(show_mat, show_mat, cv::COLOR_BGR2RGB);
        cv::circle(show_mat, cam_center_, 5, cv::Scalar(255,0,0), -1);
        cv::circle(show_mat, target_armor.center2d, 5, cv::Scalar(0,255,0), -1);
        cv::circle(show_mat, coord_solver.reproject(target_armor.center3d_predict), 5, cv::Scalar(0,0,255), -1);
        // cv::imshow("show_mat", show_mat);
		tracker_img_pub_.publish(cv_bridge::CvImage(armors_msg->header, "rgb8", show_mat).toImageMsg());
    }

// ###############tmp debug################
    // 每隔1s发布一次目标装甲板信息
    static auto tmp_last_time = this->now();
    auto tmp_now_time = this->now();
    if(tmp_now_time - tmp_last_time > 1.0s){
        RCLCPP_INFO(this->get_logger(), "###############tmp debug################");
        RCLCPP_INFO(this->get_logger(), "tmp_now_time: %f, tmp_last_time: %f", tmp_now_time.seconds(), tmp_last_time.seconds());
        RCLCPP_INFO(this->get_logger(), "center3d_world.x: %f, center3d_world.y: %f, center3d_world.z: %f", target_armor.center3d_world.x(), target_armor.center3d_world.y(), target_armor.center3d_world.z());
        RCLCPP_INFO(this->get_logger(), "center3d_predict.x: %f, center3d_predict.y: %f, center3d_predict.z: %f", target_armor.center3d_predict.x(), target_armor.center3d_predict.y(), target_armor.center3d_predict.z());
        RCLCPP_INFO(this->get_logger(), "velocity.x: %f, velocity.y: %f, velocity.z: %f", target_armor.velocity.x(), target_armor.velocity.y(), target_armor.velocity.z());
        RCLCPP_INFO(this->get_logger(), "pitch_offset: %f", pitch_offset);
        RCLCPP_INFO(this->get_logger(), "angle.x: %f, angle.y: %f", target_armor.angle.x(), target_armor.angle.y());
        RCLCPP_INFO(this->get_logger(), "eulr.x: %f, eulr.y: %f, eulr.z: %f", target_armor.euler.x(), target_armor.euler.y(), target_armor.euler.z());
        RCLCPP_INFO(this->get_logger(), "eulr2angle.x: %f, eulr2angle.y: %f, eulr2angle.z: %f", target_armor.euler.x() * 180 / CV_PI, target_armor.euler.y() * 180 / CV_PI, target_armor.euler.z() * 180 / CV_PI);
        RCLCPP_INFO(this->get_logger(), "###############tmp debug################");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), " ");
        tmp_last_time = tmp_now_time;
    }
    publishMarkers(target_armor, armors_msg->header);
    
    // 发布目标装甲板信息
    armor_interfaces::msg::TargetInfo target_info;
    target_info.header = armors_msg->header;
    target_info.id = target_armor.key[0];
    target_info.euler.x = target_armor.angle.x();
    target_info.euler.y = target_armor.angle.y();
    target_info_pub_->publish(target_info);
}

/**
 * @brief 构造追踪器函数
 */
void ArmorTrackerNode::createTrackers(){
    EKF_param param;
    param.Q(0, 0) = this->declare_parameter("ekf.Q00", 0.01);
    param.Q(1, 1) = this->declare_parameter("ekf.Q11", 0.1);
    param.Q(2, 2) = this->declare_parameter("ekf.Q22", 0.01);
    param.Q(3, 3) = this->declare_parameter("ekf.Q33", 0.1);
    param.Q(4, 4) = this->declare_parameter("ekf.Q44", 0.01);
    param.Q(5, 5) = this->declare_parameter("ekf.Q55", 0.1);

    param.R(0, 0) = this->declare_parameter("ekf.R00", 0.05);
    param.R(1, 1) = this->declare_parameter("ekf.R11", 0.05);
    param.R(2, 2) = this->declare_parameter("ekf.R22", 0.05);


    double max_lost_time_ = this->declare_parameter("max_lost_time", 0.5);
    double max_lost_distance_ = this->declare_parameter("max_lost_distance", 0.4);
    int lost_count_threshold_ = this->declare_parameter("lost_count_threshold", 10);
    int match_count_threshold_ = this->declare_parameter("match_count_threshold", 10);
    // ArmorTracker
    tracker_ptr_ = std::make_unique<ArmorTracker>(param, max_lost_time_, max_lost_distance_, lost_count_threshold_, match_count_threshold_);
}


void ArmorTrackerNode::publishMarkers(const Armor & target_armor, const std_msgs::msg::Header& header){
    position_marker_.header = header;
    velocity_marker_.header = header;
    predict_position_marker_.header = header;

    if (target_armor.is_tracking) {
        position_marker_.action = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position.x = target_armor.center3d_world.x();
        position_marker_.pose.position.y = target_armor.center3d_world.y();
        position_marker_.pose.position.z = target_armor.center3d_world.z();
        position_marker_.color.r = 0.;

        predict_position_marker_.action = visualization_msgs::msg::Marker::ADD;
        predict_position_marker_.pose.position.x = target_armor.center3d_predict.x();
        predict_position_marker_.pose.position.y = target_armor.center3d_predict.y();
        predict_position_marker_.pose.position.z = target_armor.center3d_predict.z();
        predict_position_marker_.color.r = 1.;

        velocity_marker_.action = visualization_msgs::msg::Marker::ADD;
        velocity_marker_.points.clear();
        geometry_msgs::msg::Point arrow_start;
        arrow_start.x = target_armor.center3d_world.x();
        arrow_start.y = target_armor.center3d_world.y();
        arrow_start.z = target_armor.center3d_world.z();
        velocity_marker_.points.emplace_back(arrow_start);

        geometry_msgs::msg::Point arrow_end = arrow_start;
        arrow_end.x += target_armor.velocity.x();
        arrow_end.y += target_armor.velocity.y();
        arrow_end.z += target_armor.velocity.z();
        velocity_marker_.points.emplace_back(arrow_end);
    } else {
        position_marker_.action = visualization_msgs::msg::Marker::DELETE;
        velocity_marker_.action = visualization_msgs::msg::Marker::DELETE;
        predict_position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(velocity_marker_);
    marker_array.markers.emplace_back(predict_position_marker_);
    marker_pub_->publish(marker_array);
}

/**
 * @brief 选择击打车辆ID
 * 
 * @param armors 装甲板数组
 * @return string 返回选择装甲板的key
 */
string ArmorTrackerNode::chooseTargetID(vector<Armor> &armors)
{
    for (auto armor : armors)
    {
        //若视野中存在英雄且距离小于危险距离，直接选为目标
        if (armor.id == 1 && armor.center3d_world.norm() <= hero_danger_zone)
        {
            return armor.key;
        }
    }
    //若不存在则返回队首元素key，一般情况下 已经排过序 为面积最大的装甲板key
    return (*armors.begin()).key;
}

/**
 * @brief 从装甲板中选择最终目标
 * 
 * @param armors 装甲板vector
 * @return Armor 所选取的装甲板
 */
Armor ArmorTrackerNode::chooseTargetArmor(vector<Armor> armors)
{
    //TODO:优化打击逻辑
    float max_area = 0;
    int target_idx = 0;
    int armors_size = armors.size();
    for(int i = 0; i < armors_size; i++)
    {
        auto area = calcTetragonArea(armors[i].positions2d);
        if (area >= max_area)
        {
            max_area = area;
            target_idx = i;
        }
    }
    return armors[target_idx];
}

/**
 * @brief 更新陀螺Score，函数关系在MATLAB中测试得出，在程序帧率恒定100fps
 * 的假设下，该函数关系可以在转速为 5rad/s -- 15rad/s 的情况下，
 * 在10到12次装甲板切换后识别出陀螺状态，无切换约0.5s-1s后自动退出陀螺状态
 * 
 * @return true 更新分数成功
 */
bool ArmorTrackerNode::updateSpinScore()
{    
    for (auto score = trackers_map.begin(); score != trackers_map.end();)
    {
        SpinHeading spin_status = UNKNOWN;
        
        //若trackers_map不存在该元素
        if (trackers_map.count((*score).first) != 0)
            spin_status = (*score).second.spin_heading;
        // cout<<(*score).first<<"--:"<<(*score).second<<" "<<spin_status<<endl;
        
        // RCLCPP_INFO(this->get_logger(), "Current Spin score : %s : ");
        // LOG(INFO)<<"[SpinDetection] Current Spin score :"<<(*score).first<<" : "<<(*score).second.spin_score<<" "<<spin_status;
        // 若分数过低移除此元素
        if (abs((*score).second.spin_score) <= anti_spin_judge_low_thres && spin_status != UNKNOWN)
        {
            // fmt::print(fmt::fg(fmt::color::red), "[SpinDetection] Removing {}.\n", (*score).first);
            RCLCPP_ERROR(this->get_logger(), "[SpinDetection] Removing {%s}.", (*score).first.data());
            // LOG(INFO)<<"[SpinDetection] Removing "<<(*score).first;
            // score = trackers_map.erase(score);
            ++score;
            continue;
        }
        
        if (spin_status != UNKNOWN)
            (*score).second.spin_score = 0.978 * (*score).second.spin_score - 1 * abs((*score).second.spin_score) / (*score).second.spin_score;
        else
            (*score).second.spin_score = 0.997 * (*score).second.spin_score - 1 * abs((*score).second.spin_score) / (*score).second.spin_score;
        
        //当小于该值时移除该元素
        if (abs((*score).second.spin_score) < 3 || isnan((*score).second.spin_score))
        {
            // trackers_map.erase((*score).first);
            // score = trackers_map.erase(score);
            ++score;
            continue;
        }
        else if (abs((*score).second.spin_score) >= anti_spin_judge_high_thres)
        {
            (*score).second.spin_score = anti_spin_judge_high_thres * abs((*score).second.spin_score) / (*score).second.spin_score;
            if ((*score).second.spin_score > 0)
                trackers_map[(*score).first].spin_heading = CLOCKWISE;
            else if((*score).second.spin_score < 0)
                trackers_map[(*score).first].spin_heading = COUNTER_CLOCKWISE;
        }
        ++score;
    }
    return true;
}


} // namespace armor_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ArmorTrackerNode)