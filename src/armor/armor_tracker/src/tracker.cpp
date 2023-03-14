/**
 * @file tracker.cpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-02
 * 
 */

#include "armor_tracker/tracker.h"


ArmorTracker::ArmorTracker(EKF_param& param){
    // 初始化EKF
    ekf.Q = param.Q;
    ekf.R = param.R;
    ekf.P = Eigen::Matrix<double, 6, 6>::Identity();
    ekf.init();
}

void ArmorTracker::init(Armor& src, double timestmp){
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
 * @brief 预测器
 * @param xyz 更新数据(x,y,z) / (yaw, pitch, distance)
 * @param timestmp 时间戳
 * @return std::vector<double> (x, vx, y, vy, z)
 */
Eigen::VectorXd ArmorTracker::predict_filter(double timestmp){
    Predict predictfunc;
    double delta_t = timestmp - pre_timestamp;
    pre_timestamp = timestmp;

    predictfunc.delta_t = delta_t;      // 设置距离上次预测的时间
    Eigen::Matrix<double, 6, 1> Xe = ekf.predict(predictfunc);           // 更新预测器，此时预测器里的是预测值

    Eigen::VectorXd res = Xe;
    return res;
}

/**
 * @brief 更新EKF
 * @param xyz 更新数据(x,y,z) / (yaw, pitch, distance)
 * @param timestmp 时间戳
 * @return std::vector<double> (x, vx, y, vy, z)
 */
Eigen::VectorXd ArmorTracker::update_filter(const Armor& src){
    Measure measure;

    Eigen::Matrix<double, 6, 1> Xr;
    Xr << src.center3d_world[0], 0, src.center3d_world[1], 0, src.center3d_world[2], 0;

    std::cout<<"Xr: "<<Xr.transpose()<<std::endl;

    Eigen::Matrix<double, 3, 1> Yr;
    measure(Xr.data(), Yr.data());      // 转化成球面坐标 Yr

    std::cout<<"Yr: "<<Yr.transpose()<<std::endl;

    Eigen::Matrix<double, 6, 1> Xe = ekf.update(measure, Yr);   // 更新滤波器，输入真实的球面坐标 Yr

    std::cout<<"Xe: "<<Xe.transpose()<<std::endl;

    pre_armor = src;

    Eigen::VectorXd res = Xe;
    return res;
}

/**
 * @brief 进行跟踪装甲板，更新建议击打装甲板
 * @param timestmp 
 */
void ArmorTracker::update(const std::vector<Armor> & src, double timestmp){

    suggest_fire = false;
    bool matched = false;

    if( timestmp - pre_timestamp < max_lost_time &&
        (tracker_state == TRACKING || tracker_state == TEMP_LOST) ){
            Eigen::VectorXd ekf_predict = predict_filter(timestmp);           // 预测值
            Eigen::Vector3d ekf_prediction(ekf_predict[0], ekf_predict[2], ekf_predict[4]);  // 预测位置
            // 遍历所有装甲板，找到最近的装甲板
            double min_distance = 1e9;
            for (auto armor : src){
                double distance = (armor.center3d_world - ekf_prediction).norm();
                if (distance < max_lost_distance && distance < min_distance){
                    matched = true;
                    suggest_armor = armor;
                    min_distance = distance;
                }
            }
            if(!matched)
                tracker_state = LOST;
    }
    else
        tracker_state = LOST;
    
    if(tracker_state == LOST || tracker_state == DETECTING) {
        // 遍历所有装甲板，找到面积最大的装甲板
        double max_area = 0;
        for (auto armor : src){
            if (armor.area > max_area){
                matched = true;
                suggest_armor = armor;
                max_area = armor.area;
            }
        }
        if(matched){
            init(suggest_armor, timestmp);
            predict_filter(timestmp);
            tracker_state = DETECTING;
        }
    }
    if(matched){
        Eigen::VectorXd ekf_update = update_filter(suggest_armor);           // 更新值
        Eigen::Vector3d ekf_update_position(ekf_update[0], ekf_update[2], ekf_update[4]);  // 更新位置
        std::cout<<"update_position: "<<ekf_update_position.transpose()<<std::endl;

        // 更新建议击打装甲板
        suggest_armor.predict = ekf_update_position;
        suggest_fire = true;
    }
    else {
        suggest_fire = false;
    }

    // 状态机
    if(tracker_state == DETECTING){
        if(matched){
            matched_count++;
            if(matched_count > matched_count_threshold)
                tracker_state = TRACKING;
        }
        else{
            matched_count = 0;
            tracker_state = LOST;
        }
    }
    else if(tracker_state == TRACKING){
        if(!matched){
            tracker_state = TEMP_LOST;
            lost_count = 1;
        }
    }
    else if(tracker_state == TEMP_LOST){
        if(matched){
            tracker_state = TRACKING;
            lost_count = 0;
        }
        else{
            lost_count++;
            if(lost_count > lost_count_threshold)
                tracker_state = LOST, lost_count = 0;
        }
    }
    else if(tracker_state == LOST)
        tracker_state = DETECTING;
}

/**
 * @brief 获取建议击打装甲板
 * @param target_armor 
 * @return true 
 * @return false 
 */
bool ArmorTracker::getTargetArmor(Armor& target_armor){
    if(suggest_fire){
        target_armor = suggest_armor;
        return true;
    }
    else
        return false;
}