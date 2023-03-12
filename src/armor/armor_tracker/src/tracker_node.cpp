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

    auto pkg_path = ament_index_cpp::get_package_share_directory("armor_tracker");
    coord_solver.loadParam(pkg_path+"/params/coord_param.yaml", "KE0200110076");
    
    target_info_pub_ = this->create_publisher<armor_interfaces::msg::TargetInfo>("/processor/target", rclcpp::SensorDataQoS());
    
    armors_sub_ = this->create_subscription<armor_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),std::bind(&ArmorTrackerNode::armorsCallback, this, std::placeholders::_1));
}

void ArmorTrackerNode::armorsCallback(armor_interfaces::msg::Armors::ConstSharedPtr armors_msg){
    RCLCPP_INFO(this->get_logger(), "recvice armors!");

    double time_now = 1.0 * armors_msg->header.stamp.nanosec + armors_msg->header.stamp.sec*1e-9;

    // TODO:
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
    // TargetType target_type = SMALL;
    if(!armors.empty()){
        armor_interfaces::msg::TargetInfo target_info;
        target_info.header = armors_msg->header;
        target_info.id = armors[0].key[0];
        target_info.euler.x = armors[0].center3d_cam.x();
        target_info.euler.y = armors[0].center3d_cam.y();
        target_info.euler.z = armors[0].center3d_cam.z();
        target_info_pub_->publish(target_info);
    }

    // TODO:拍完中期形态视频之后优化
    return ;

    
    ///------------------------生成/分配ArmorTracker----------------------------
    new_armors_cnt_map.clear();
    for(auto & armor : armors){
        string tracker_key;
        tracker_key = armor.key;
        int armor_tracker_num = 0;
        if(trackers_map.find(tracker_key) != trackers_map.end())
            armor_tracker_num = trackers_map[tracker_key].armor_tracker.size();
        
        //当不存在该类型装甲板ArmorTracker
        if (armor_tracker_num == 0)
        {
            TrackerParams tracker_params;
            tracker_params.armor_tracker.push_back(ArmorTracker(armor, armor.time_stamp));
            tracker_params.spin_heading = UNKNOWN;
            tracker_params.spin_score = 0;
            // ArmorTracker tracker(armor.time_stamp);
            trackers_map.insert(make_pair(armor.key, tracker_params));
            new_armors_cnt_map[armor.key]++;
        }
        //当存在一个该类型ArmorTracker
        else if(armor_tracker_num == 1){
            auto candidate = trackers_map.find(armor.key);
            double delta_t = time_now - (*candidate).second.armor_tracker[0].pre_timestamp;
            auto delta_dist = (armor.center3d_world - (*candidate).second.armor_tracker[0].pre_armor.center3d_world).norm();
            auto velocity = (delta_dist / delta_t) * 1e3;
            // 若匹配则使用此ArmorTracker
            if (velocity <= max_v)
            {
                (*candidate).second.armor_tracker[0].predict(armor, time_now);
                (*candidate).second.armor_tracker[0].update(armor, time_now);
            }
            //若不匹配则创建新ArmorTracker
            else
            {
                TrackerParams tracker_params;
                tracker_params.armor_tracker.push_back(ArmorTracker(armor, armor.time_stamp));
                tracker_params.spin_heading = UNKNOWN;
                tracker_params.spin_score = 0;
                // ArmorTracker tracker(armor.time_stamp);
                trackers_map.insert(make_pair(armor.key, tracker_params));
                new_armors_cnt_map[armor.key]++;
            }
        }
        //当存在多个该类型装甲板ArmorTracker
        else
        {
            //1e9无实际意义，仅用于以非零初始化
            double min_v = 1e9;
            int min_delta_t = 1e9;
            bool is_best_candidate_exist = false;
            int best_candidate;
            auto candiadates = trackers_map.find(armor.key);
            //遍历所有同Key预测器，匹配速度最小且更新时间最近的ArmorTracker
            for (auto iter = candiadates->second.armor_tracker.begin(); iter != candiadates->second.armor_tracker.begin(); ++iter)
            {
                auto delta_t = time_now - (*iter).pre_timestamp;
                auto delta_dist = (armor.center3d_world - (*iter).pre_armor.center3d_world).norm();
                auto velocity = (delta_dist / delta_t) * 1e3;
                if (velocity <= max_v && velocity <= min_v && delta_t <= min_delta_t)
                {
                    min_delta_t = delta_t;
                    min_v = velocity;
                    best_candidate = iter - candiadates->second.armor_tracker.begin();
                    is_best_candidate_exist = true;
                }
            }
            if (is_best_candidate_exist)
            {
                // auto velocity = min_v;
                // auto delta_t = min_delta_t;
                candiadates->second.armor_tracker[best_candidate].predict(armor, time_now);
                candiadates->second.armor_tracker[best_candidate].update(armor, time_now);
            }
            else
            {
                TrackerParams tracker_params;
                tracker_params.armor_tracker.push_back(ArmorTracker(armor, armor.time_stamp));
                tracker_params.spin_heading = UNKNOWN;
                tracker_params.spin_score = 0;
                // ArmorTracker tracker(armor.time_stamp);
                trackers_map.insert(make_pair(armor.key, tracker_params));
                new_armors_cnt_map[armor.key]++;
            }

        }
    }
    
    // FIXME:这里用用迭代器删除出现bug
    // for (auto iter = trackers_map.begin(); iter != trackers_map.end(); iter++)
    // {
    //     //删除过久之前的装甲板
    //     for (auto iter_tracker = iter->second.armor_tracker.begin(); iter_tracker != iter->second.armor_tracker.end();)
    //     {
    //         if (time_now - (*iter_tracker).pre_timestamp > max_delta_t)
    //             iter_tracker = iter->second.armor_tracker.erase(iter_tracker);
    //         else
    //             ++iter_tracker;
    //     }
    // }

#ifndef USING_SPIN_DETECT
    ///------------------------检测装甲板变化情况,计算各车陀螺分数----------------------------
    for (auto cnt : new_armors_cnt_map)
    {
        if (cnt.second == 1)
        {
            auto tracker = trackers_map.find(cnt.first);
            if(tracker == trackers_map.end()) continue;
            auto armor_tracker_num = tracker->second.armor_tracker.size();
            //当存在同时存在两块同类别装甲板Tracker时才进入陀螺状态识别
            if (armor_tracker_num == 2)
            {
                double last_armor_center;
                double last_armor_timestamp;
                double new_armor_center;
                double new_armor_timestamp;
                auto candidate = tracker->second.armor_tracker.begin();
                //确定新增装甲板与历史装甲板
                if ((*candidate).is_initialized)
                {
                    last_armor_center = (*candidate).last_armor.center3d_cam[0];
                    last_armor_timestamp = (*candidate).last_timestamp;
                    ++candidate;
                    if (!(*candidate).is_initialized)
                        {
                            new_armor_center = (*candidate).last_armor.center3d_cam[0];
                            new_armor_timestamp = (*candidate).last_timestamp;
                        }
                    else
                        continue;
                }
                else
                {
                    new_armor_center = (*candidate).last_armor.center3d_cam[0];
                    new_armor_timestamp = (*candidate).last_timestamp;
                    ++candidate;
                    if ((*candidate).is_initialized)
                        {
                            last_armor_center = (*candidate).last_armor.center3d_cam[0];
                            last_armor_timestamp = (*candidate).last_timestamp;
                        }
                    else
                        continue;
                }
                auto spin_movement = new_armor_center - last_armor_center;
                // cout<<last_armor_timestamp<<" : "<<new_armor_timestamp<<endl;
                if (tracker->second.spin_score == 0 && abs(spin_movement) > 0.05 && last_armor_timestamp == new_armor_timestamp)
                    tracker->second.spin_score = 100 * spin_movement / abs(spin_movement);
                else if (abs(spin_movement) > 0.05 && last_armor_timestamp == new_armor_timestamp)
                    tracker->second.spin_score = 2 * tracker->second.spin_score;
            }
        }
    }
    ///------------------更新反陀螺socre_map，更新各车辆陀螺状态-----------------------------
    updateSpinScore();
    // cout<<"-----------------------"<<endl;
    // for (auto status : spin_status_map)
    // {
    //     cout<<status.first<<" : "<<status.second<<endl;
    // }
#endif //USING_SPIN_DETECT

    ///-----------------------------判断击打车辆------------------------------------------
    auto target_id = chooseTargetID(armors);
    auto ID_candiadates = trackers_map.find(target_id);
    ///---------------------------获取最终装甲板序列---------------------------------------
    bool is_target_spinning;
    Armor target;
    Eigen::Vector3d aiming_point;
    std::vector<ArmorTracker*> final_trackers;
    std::vector<Armor> final_armors;
    //TODO:反陀螺防抖(增加陀螺模式与常规模式)
    //若目标处于陀螺状态，预先瞄准目标中心，待预测值与该点距离较近时开始击打
    SpinHeading spin_status;
    spin_status = ID_candiadates->second.spin_heading;
    if (spin_status != UNKNOWN)
        is_target_spinning = true;
    else
        is_target_spinning = false;
    if( is_target_spinning ) ; // ros2不使用会报错 没有实际意义
    ///----------------------------------反陀螺击打---------------------------------------
    if (spin_status != UNKNOWN)
    {
        //------------------------------尝试确定旋转中心-----------------------------------
        // auto available_candidates_cnt = 0;
        for (auto iter = ID_candiadates->second.armor_tracker.begin(); iter != ID_candiadates->second.armor_tracker.end(); ++iter)
        {
            final_armors.push_back((*iter).last_armor);
            //若Tracker未完成初始化，不考虑使用
            // if (!(*iter).second.is_initialized || (*iter).second.history_info.size() < 3)
            // {
            //     continue;
            // }
            // else
            // {
            //     final_trackers.push_back(&(*iter).second);
            //     available_candidates_cnt++;
            // }
        }
        // if (available_candidates_cnt == 0)
        // {
        //     cout<<"Invalid"<<endl;   
        // }
        // else
        // {   //TODO:改进旋转中心识别方法
        //     //FIXME:目前在目标小陀螺时并移动时，旋转中心的确定可能存在问题，故该语句块中的全部计算结果均暂未使用
        //     //-----------------------------计算陀螺旋转半径--------------------------------------
        //     Eigen::Vector3d rotate_center_cam = {0,0,0};
        //     Eigen::Vector3d rotate_center_car = {0,0,0};
        //     for(auto tracker : final_trackers)
        //     {
        //         std::vector<Eigen::Vector3d> pts;
        //         for (auto pt : tracker->history_info)
        //         {
        //             pts.push_back(pt.center3d_world);
        //         }
        //         auto sphere = FitSpaceCircle(pts);
        //         auto radius = sphere[3];
        //         if (tracker->radius == 0)
        //             tracker->radius = radius;
        //         else//若不为初值，尝试进行半径平均以尽量误差
        //             tracker->radius = (tracker->radius + radius) / 2;
        //         //-----------------------------计算陀螺中心与预瞄点-----------------------------------
        //         //此处世界坐标系指装甲板世界坐标系，而非车辆世界坐标系
        //         Eigen::Vector3d rotate_center_world = {0,
        //                             sin(25 * 180 / CV_PI) * tracker->radius,
        //                             - cos(25 * 180 / CV_PI) * tracker->radius};
        //         auto rotMat = eulerToRotationMatrix(tracker->prev_armor.euler);
        //         //Pc = R * Pw + T
        //         rotate_center_cam = (rotMat * rotate_center_world) + tracker->prev_armor.center3d_cam;
        //         rotate_center_car += coordsolver.worldToCam(rotate_center_cam, rmat_imu);
        //     }
        //     //求解旋转中心
        //     rotate_center_car /= final_trackers.size();
        // }
        //若存在一块装甲板
        if (final_armors.size() == 1)
        {
            target = final_armors.at(0);
        }
        //若存在两块装甲板
        else if (final_armors.size() == 2)
        {
            //对最终装甲板进行排序，选取与旋转方向相同的装甲板进行更新
            sort(final_armors.begin(),final_armors.end(),[](Armor& prev, Armor& next)
                                {return prev.center3d_cam[0] < next.center3d_cam[0];});
            //若顺时针旋转选取右侧装甲板更新
            if (spin_status == CLOCKWISE)
                target = final_armors.at(1);
            //若逆时针旋转选取左侧装甲板更新
            else if (spin_status == COUNTER_CLOCKWISE)
                target = final_armors.at(0);
        }
#ifdef USING_PREDICT
        Eigen::Vector3d predict_value;
        auto delta_t = src.timestamp - last_timestamp;
        auto delta_dist = (target.center3d_world - last_armor.center3d_world).norm();
        auto velocity = (delta_dist / delta_t) * 1e3;
        if (target.key != last_armor.key || velocity > max_v)
        {
            predictor.initParam(predictor_param_loader);
            aiming_point = target.center3d_cam;
        }
        else
        {
            auto aiming_point_world = predictor.predict(target.center3d_world, src.timestamp);
            // aiming_point = aiming_point_world;
            aiming_point = coordsolver.worldToCam(aiming_point_world, rmat_imu);
        }
#else
    // aiming_point = coordsolver.worldToCam(target.center3d_world,rmat_imu);
    aiming_point = target.center3d_cam;
#endif //USING_PREDICT
    }
    ///----------------------------------常规击打---------------------------------------
    else
    {
        for (auto iter = ID_candiadates->second.armor_tracker.begin(); iter != ID_candiadates->second.armor_tracker.begin(); ++iter)
        {
            final_armors.push_back((*iter).last_armor);
        }
        //选取最大的装甲板进行击打
        target = chooseTargetArmor(final_armors);

#ifdef USING_PREDICT
        auto delta_t = src.timestamp - last_timestamp;
        auto delta_dist = (target.center3d_world - last_armor.center3d_world).norm();
        auto velocity = (delta_dist / delta_t) * 1e3;

        if (target.key != last_armor.key || velocity > max_v)
        {
            predictor.initParam(predictor_param_loader);
            aiming_point = target.center3d_cam;
        }
        else
        {
            auto aiming_point_world = predictor.predict(target.center3d_world, src.timestamp);
            // aiming_point = aiming_point_world;
            aiming_point = coordsolver.worldToCam(aiming_point_world, rmat_imu);
        }
#else
    // aiming_point = coordsolver.worldToCam(target.center3d_world,rmat_imu);
    aiming_point = target.center3d_cam;
#endif //USING_PREDICT
    }
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