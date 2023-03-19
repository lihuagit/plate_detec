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

    // 是否调试
	debug_ = this->declare_parameter("debug", true);
	tracker_img_pub_ = image_transport::create_publisher(this, "/tracker_img");

    // 位姿解算器
    auto pkg_path = ament_index_cpp::get_package_share_directory("armor_tracker");
    coord_solver.loadParam(pkg_path+"/params/coord_param.yaml", "KE0200110076");
    coord_solver.bullet_speed = shoot_v;

    // tracker
    createTrackers();

    target_info_pub_ = this->create_publisher<armor_interfaces::msg::TargetInfo>("/processor/target", rclcpp::SensorDataQoS());
    
    armors_sub_ = this->create_subscription<armor_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),std::bind(&ArmorTrackerNode::armorsCallback, this, std::placeholders::_1));
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
    }

    // Tracker
    bool is_tracking = false;
    tracker_ptr_->update(armors, time_now);

    // double delta_t = ;
    Eigen::Vector3d target_center3d;
    is_tracking = tracker_ptr_->getTargetArmor(shoot_v, target_center3d);
    RCLCPP_INFO(this->get_logger(), "is_tracking: %d", is_tracking);

    Armor target_armor;

    if(is_tracking)
        target_armor = tracker_ptr_->pre_armor;
    else
        return ;
        
    auto pitch_offset = coord_solver.dynamicCalcPitchOffset(target_armor.center3d_world);
    // auto pitch_offset = geiPitch(target_armor.predict);
    target_armor.angle = coord_solver.calcYawPitch(target_armor.predict);
    target_armor.angle.y() += pitch_offset;

    if(debug_){
        RCLCPP_INFO(this->get_logger(), "target_armor.center3d_world.x: %f, target_armor.center3d_world.y: %f, target_armor.center3d_world.z: %f", target_armor.center3d_world.x(), target_armor.center3d_world.y(), target_armor.center3d_world.z()); 
        RCLCPP_INFO(this->get_logger(), "target_armor.predict.x: %f, target_armor.predict.y: %f, target_armor.predict.z: %f", target_armor.predict.x(), target_armor.predict.y(), target_armor.predict.z());
        RCLCPP_INFO(this->get_logger(), "pitch_offset: %f", pitch_offset);
        RCLCPP_INFO(this->get_logger(), "target_armor.angle.x: %f, target_armor.angle.y: %f", target_armor.angle.x(), target_armor.angle.y());
        
        cv::Mat show_mat(1280,1024,CV_8UC3, cv::Scalar(255,255,255));
		cv::cvtColor(show_mat, show_mat, cv::COLOR_BGR2RGB);
        cv::circle(show_mat, cam_center_, 5, cv::Scalar(255,0,0), -1);
        cv::circle(show_mat, target_armor.center2d, 5, cv::Scalar(0,255,0), -1);
        cv::circle(show_mat, coord_solver.reproject(target_armor.predict), 5, cv::Scalar(0,0,255), -1);
        cv::imshow("show_mat", show_mat);
		tracker_img_pub_.publish(cv_bridge::CvImage(armors_msg->header, "rgb8", show_mat).toImageMsg());
    }
    
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