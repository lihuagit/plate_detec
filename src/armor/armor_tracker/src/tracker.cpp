/**
 * @file tracker.cpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-02
 * 
 */

#include "armor_tracker/tracker.h"


ArmorTracker::ArmorTracker( EKF_param param, double max_lost_time_, double max_lost_distance_,
                            int lost_count_threshold_, int match_count_threshold_)
                            : max_lost_time(max_lost_time_), max_lost_distance(max_lost_distance_),
                            lost_count_threshold(lost_count_threshold_), matched_count_threshold(match_count_threshold_)
{
    // 初始化EKF
    ekf.Q = param.Q;
    ekf.R = param.R;
    tracker_state = DETECTING;
}

void ArmorTracker::init(const Armor& src, double timestmp){
    pre_armor = src;
    pre_timestamp = timestmp;
    Eigen::Matrix<double, 6, 1> Xe;
    // Xe(0, 0) = src.center3d_world(0, 0);
    // Xe(1, 0) = ekf.Xe(1, 0);
    // Xe(2, 0) = src.center3d_world(1, 0);
    // Xe(3, 0) = ekf.Xe(3, 0);
    // Xe(4, 0) = src.center3d_world(2, 0);
    // Xe(5, 0) = ekf.Xe(5, 0);

    Xe(0, 0) = src.center3d_world(0, 0);
    Xe(1, 0) = 0;
    Xe(2, 0) = src.center3d_world(1, 0);
    Xe(3, 0) = 0;
    Xe(4, 0) = src.center3d_world(2, 0);
    Xe(5, 0) = 0;

    ekf.init(Xe);
}

/**
 * @brief 进行跟踪装甲板，更新建议击打装甲板
 * @param timestmp 
 */
void ArmorTracker::update(const std::vector<Armor> & src, double timestmp){
    // FIXME: x坐标以及x的速度vx会出现异常值，导致预测值出现异常，未找到原因，暂时强行解决
    if(ekf.Xe(0, 0) > 1000 || ekf.Xe(0, 0) < -1000 || ekf.Xe(1, 0) > 1000 || ekf.Xe(1, 0) < -1000){
        ekf.reset();
        
        tracker_state = DETECTING;
        matched_count = 0;
        std::cout<<"x or vx is abnormal, reset ekf"<<std::endl;
        return;
    }

    // 1. 如果时间戳差值大于最大丢失时间，状态置为DETECTING
    if(abs(timestmp - pre_timestamp) > max_lost_time){
        tracker_state = DETECTING;
        matched_count = 0;
    }
    std::cout << "timestamp - pre_timestamp: " << abs(timestmp - pre_timestamp) << std::endl;
    pre_timestamp = timestmp;

    // 2. 寻找最优装甲板
    // 如果状态为TRACKING、TEMP_LOST，选择最优装甲板，与预测值进行匹配
    double best_condition = 1e9;
    int best_index = -1;
    int src_size = src.size();
    if( tracker_state == TRACKING || tracker_state == TEMP_LOST){
        // 预测
        Predict predictfunc;
        double delta_t = timestmp - pre_timestamp;
        pre_timestamp = timestmp;

        predictfunc.delta_t = delta_t;      // 设置距离上次预测的时间
        Eigen::Matrix<double, 6, 1> Xe = ekf.predict(predictfunc);           // 更新预测器，此时预测器里的是预测值
        Eigen::Vector3d Xe_world;
        Xe_world << Xe(0, 0), Xe(2, 0), Xe(4, 0);

        // 按照与预测值的距离，选择最优装甲板
        best_condition = 1e9;
        for(int i = 0; i < src_size; i++){
            double dist = (src[i].center3d_world - Xe_world).norm();
            if(dist < best_condition){
                best_condition = dist;
                best_index = i;
            }
        }

        // 如果没有找到，按照armor_key进行匹配
        if(best_index == -1)
            for(int i = 0; i < src_size; i++)
                if(src[i].key == pre_armor.key){
                    best_index = i;
                    break;
                }
    }
    // 如果状态为DETECTING，按照最大area进行匹配
    else if(tracker_state == DETECTING){
        best_condition = 0;
        for(int i = 0; i < src_size; i++)
            if(src[i].area > best_condition){
                best_condition = src[i].area;
                best_index = i;
            }
        // 如果找到了 对ekf进行初始化
        if(best_index != -1)
            init(src[best_index], timestmp);
    }

    // 3. 如果找到了最优装甲板，进行更新
    if(best_index != -1){
        pre_armor = src[best_index];

        // 如果为追踪状态，则更新ekf
        if(tracker_state == TRACKING || tracker_state == TEMP_LOST){
            Measure measure;
            Eigen::Matrix<double, 6, 1> Xr;
            Xr << src[best_index].center3d_world[0], 0, src[best_index].center3d_world[1], 0, src[best_index].center3d_world[2], 0;
            Eigen::Matrix<double, 3, 1> Yr;
            measure(Xr.data(), Yr.data());      // 转化成球面坐标 Yr
            Eigen::Matrix<double, 6, 1> Xe = ekf.update(measure, Yr);   // 更新滤波器，输入真实的球面坐标 Yr

            std::cout << "Xe: " << Xe.transpose() << std::endl;
            std::cout << "Yr: " << Yr.transpose() << std::endl;

            // 更新建议击打装甲板
            pre_armor.predict << Xe(0, 0), Xe(2, 0), Xe(4, 0);

            std::cout << "pre_armor.predict: " << pre_armor.predict.transpose() << std::endl;
        }
    }

    // 4. 更新状态
    if(best_index != -1){
        // 如果状态为DETECTING，更新为TRACKING
        if(tracker_state == DETECTING){
            matched_count ++;
            if(matched_count > matched_count_threshold)
                tracker_state = TRACKING;
            else 
                tracker_state = DETECTING;
        }
        // 如果状态为TRACKING，更新为TRACKING
        else if(tracker_state == TRACKING)
            tracker_state = TRACKING;
        // 如果状态为TEMP_LOST，更新为TRACKING
        else if(tracker_state == TEMP_LOST)
            tracker_state = TRACKING;
    }
    else{
        // 如果状态为DETECTING，更新为DETECTING
        if(tracker_state == DETECTING)
            tracker_state = DETECTING;
        // 如果状态为TRACKING，更新为TEMP_LOST
        else if(tracker_state == TRACKING)
            tracker_state = TEMP_LOST;
        // 如果状态为TEMP_LOST，更新为TEMP_LOST
        else if(tracker_state == TEMP_LOST){
            lost_count ++;
            if(lost_count > lost_count_threshold)
                tracker_state = DETECTING;
            else 
                tracker_state = TEMP_LOST;
        }
    }
}

/**
 * @brief 获取预测值
 * @param shoot_v 预测速度
 * @param target_center3d 获取的预测值
 * @return true 获取预测值成功
 * @return false 获取预测值失败
 */
bool ArmorTracker::getTargetArmor(double shoot_v, Eigen::Vector3d& target_center3d){
    if(tracker_state == TRACKING || tracker_state == TEMP_LOST){
        // 预测
        Predict predictfunc;
        // double dis = ceres::sqrt(pre_armor.center3d_world[0] * pre_armor.center3d_world[0] + pre_armor.center3d_world[2] * pre_armor.center3d_world[2]);
        // double delta_t = dis / (shoot_v * (dis / pre_armor.center3d_world.norm()));
        double delta_t = pre_armor.center3d_world.norm() / shoot_v;
        std::cout << "delta_t: " << delta_t << std::endl;
        predictfunc.delta_t = delta_t;      // 设置距离上次预测的时间
        Eigen::Matrix<double, 6, 1> Xe = ekf.predict(predictfunc);           // 更新预测器，此时预测器里的是预测值
        target_center3d << Xe(0, 0), Xe(2, 0), Xe(4, 0);
        pre_armor.predict = target_center3d;
        return true;
    }
    else
        return false;
}