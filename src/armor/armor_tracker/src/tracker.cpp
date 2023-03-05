/**
 * @file tracker.cpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-02
 * 
 */

#include "armor_tracker/tracker.h"


ArmorTracker::ArmorTracker(Armor src, double timestmp){
    last_armor = src;
    last_timestamp = timestmp;
    is_initialized = false;

    history_info.push_back(src);
}

/**
 * @brief 预测器
 * @param xyz 更新数据(x,y,z) / (yaw, pitch, distance)
 * @param timestmp 时间戳
 * @return std::vector<double> (x, vx, y, vy, z)
 */
std::vector<double> ArmorTracker::predict(const Armor& src, double timestmp){
    Predict predictfunc;
    Measure measure;
    last_armor = src;
    double delta_t = timestmp - pre_timestamp;

    Eigen::Matrix<double, 5, 1> Xr;
    Xr << last_armor.center3d_world[0], 0, last_armor.center3d_world[1], 0, last_armor.center3d_world[2];
    Eigen::Matrix<double, 3, 1> Yr;
    measure(Xr.data(), Yr.data());      // 转化成相机求坐标系 Yr
    predictfunc.delta_t = delta_t;      // 设置距离上次预测的时间
    Eigen::Matrix<double, 5, 1> Xe = ekf.predict(predictfunc);           // 更新预测器，此时预测器里的是预测值

    std::vector<double> res;
    for(int i=0; i<5; i++)
        res.push_back(Xe(i, 0));
    return res;
}

/**
 * @brief 更新EKF
 * @param xyz 更新数据(x,y,z) / (yaw, pitch, distance)
 * @param timestmp 时间戳
 * @return std::vector<double> (x, vx, y, vy, z)
 */
std::vector<double> ArmorTracker::update(const Armor& src, double timestmp){
    // Predict predictfunc;
    Measure measure;
    pre_armor = last_armor;
    last_armor = src;
    // double delta_t = timestmp - pre_timestamp;
    pre_timestamp = timestmp;

    Eigen::Matrix<double, 5, 1> Xr;
    Xr << last_armor.center3d_world[0], 0, last_armor.center3d_world[1], 0, last_armor.center3d_world[2];
    Eigen::Matrix<double, 3, 1> Yr;
    measure(Xr.data(), Yr.data());      // 转化成相机求坐标系 Yr
    // predictfunc.delta_t = delta_t;      // 设置距离上次预测的时间
    Eigen::Matrix<double, 5, 1> Xe = ekf.update(measure, Yr);   // 更新滤波器，输入真实的球面坐标 Yr

    std::vector<double> res;
    for(int i=0; i<5; i++)
        res.push_back(Xe(i, 0));
    return res;
}