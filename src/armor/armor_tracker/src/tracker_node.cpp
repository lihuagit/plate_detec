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

    // 位姿解算器
    auto pkg_path = ament_index_cpp::get_package_share_directory("armor_tracker");
    coord_solver.loadParam(pkg_path+"/params/coord_param.yaml", "KE0200110076");

    // 追踪器
    EKF_param ekf_param;
    ekf_param.Q(0, 0) = 0.01;
    ekf_param.Q(1, 1) = 10;
    ekf_param.Q(2, 2) = 0.01;
    ekf_param.Q(3, 3) = 10;
    ekf_param.Q(4, 4) = 0.01;
    ekf_param.Q(5, 5) = 10;

    ekf_param.R(0, 0) = 1;
    ekf_param.R(1, 1) = 1;
    ekf_param.R(2, 2) = 800;

    tracker_ = std::make_unique<ArmorTracker>(ekf_param);

    target_info_pub_ = this->create_publisher<armor_interfaces::msg::TargetInfo>("/processor/target", rclcpp::SensorDataQoS());
    
    armors_sub_ = this->create_subscription<armor_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),std::bind(&ArmorTrackerNode::armorsCallback, this, std::placeholders::_1));
}

void ArmorTrackerNode::armorsCallback(armor_interfaces::msg::Armors::ConstSharedPtr armors_msg){
    RCLCPP_INFO(this->get_logger(), "recvice armors!");

    double time_now = 1.0 * armors_msg->header.stamp.nanosec + armors_msg->header.stamp.sec*1e-9;

    cam_center_ = cv::Point2f(coord_solver.intrinsic_cpy.at<float>(0, 2), coord_solver.intrinsic_cpy.at<float>(1, 2));

    std::vector<Armor> armors;
    for(const auto & armor_msg : armors_msg->armors){
        Armor armor;
        armor.id = armor_msg.number;
        for(auto & point : armor_msg.positions){
            // armor.positions2d.push_back(cv::Point2f(point.x, -1*point.y) - cam_center_);
            armor.positions2d.push_back(cv::Point2f(point.x, point.y));
            RCLCPP_INFO(this->get_logger(), "point.x: %f, point.y: %f", point.x, point.y);
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
        Eigen::Matrix3d rmat_imu = src.quat.toRotationMatrix();
        // auto vec = rotationMatrixToEulerAngles(rmat_imu);
        // cout<<"Euler : "<<vec[0] * 180.f / CV_PI<<" "<<vec[1] * 180.f / CV_PI<<" "<<vec[2] * 180.f / CV_PI<<endl;
    #else
        Eigen::Matrix3d rmat_imu = Eigen::Matrix3d::Identity();
    #endif //USING_IMU
    
    //进行PnP，目标较少时采取迭代法，较多时采用IPPE
    // int pnp_method;
    // if (armors.size() <= 2)
    //     pnp_method = SOLVEPNP_ITERATIVE;
    // else
    //     pnp_method = SOLVEPNP_IPPE;

    for(auto & armor : armors){ 
        PnPInfo pnp_result = coord_solver.pnp(armor.positions2d, rmat_imu, SOLVEPNP_IPPE);
        armor.center3d_world = pnp_result.armor_world;
        armor.center3d_cam = pnp_result.armor_cam;
        armor.euler = pnp_result.euler;
    }

    // TODO:拍完中期形态视频之后优化
    return ;
    bool is_track = updateTracker(armors, time_now);
    bool is_get_armor = false;

    Armor target_armor;
    if(is_track)
        is_get_armor = tracker_->getTargetArmor(target_armor);

    // TargetType target_type = SMALL;
    if(is_get_armor){
        armor_interfaces::msg::TargetInfo target_info;
        target_info.header = armors_msg->header;
        target_info.id = target_armor.key[0];
        target_info.euler.x = target_armor.predict.x();
        target_info.euler.y = target_armor.predict.y();
        target_info.euler.z = target_armor.predict.z();
        target_info_pub_->publish(target_info);
    }

}

/**
 * @brief 更新装甲板跟踪器
 * @param armors 用于更新的装甲板数组
 * @param timestmp 时间戳
 * @return true 成功更新
 * @return false 更新失败
 */
bool ArmorTrackerNode::updateTracker(const std::vector<Armor> &armors, double timestmp){
    //若装甲板数组为空，直接返回
    if(armors.empty())
        return false;
    bool is_updated = false;
    tracker_->update(armors, timestmp);
    is_updated = tracker_->suggest_fire;
    return is_updated;
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