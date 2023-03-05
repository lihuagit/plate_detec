/**
 * @file tracker.h
 * @brief 对预测进行封装，目前使用EKF
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-01
 * 
 */
#ifndef ARMOR_TRACKER__COORDSOLVER_HPP
#define ARMOR_TRACKER__COORDSOLVER_HPP
// c++
#include <vector>
#include <string>
#include <deque>

// opencv
#include <opencv2/core.hpp>

// user
#include "armor_tracker/EKF.hpp"

/**
 * @brief 此处定义匀速直线运动模型
 */
struct Predict {
    template<class T>
    void operator()(const T x0[5], T x1[5]) {
        x1[0] = x0[0] + delta_t * x0[1];  //0.1
        x1[1] = x0[1];  //100
        x1[2] = x0[2] + delta_t * x0[3];  //0.1
        x1[3] = x0[3];  //100
        x1[4] = x0[4];  //0.01
    }
    double delta_t;
};

template<class T>
void xyz2pyd(T xyz[3], T pyd[3]) {
    /*
     * 工具函数：将 xyz 转化为 pitch、yaw、distance
     */
    pyd[0] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]));  // pitch
    pyd[1] = ceres::atan2(xyz[1], xyz[0]);  // yaw
    pyd[2] = ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);  // distance
}

struct Measure {
    /*
     * 工具函数的类封装
     */
    template<class T>
    void operator()(const T x[5], T y[3]) {
        T x_[3] = {x[0], x[2], x[4]};
        xyz2pyd(x_, y);
    }
};

/**
 * @brief 记录装甲板坐标信息
 */
struct Armor
{
    char id;
    double time_stamp;
    std::string key;
    std::vector<cv::Point2f> positions2d;
    cv::Point2f center2d;
    Eigen::Vector3d center3d_cam;
    Eigen::Vector3d center3d_world;
    Eigen::Vector3d euler;
    Eigen::Vector3d predict;
};

class ArmorTracker{
private:
    AdaptiveEKF<5, 3> ekf;  // 创建ekf

public:
    ArmorTracker(Armor src, double timestmp);
    Armor pre_armor;               //上一次装甲板
    Armor last_armor;            //本次装甲板
    bool is_initialized;
    double pre_timestamp;             //上次装甲板时间戳
    double last_timestamp;          //本次装甲板时间戳
    const int max_history_len = 4;  //历史信息队列最大长度
    double velocity;
    double radius;

    std::deque<Armor> history_info;//目标队列

    std::vector<double> predict(const Armor& src, double timestmp);
    std::vector<double> update(const Armor& src, double timestmp);
};

#endif // ARMOR_TRACKER__COORDSOLVER_HPP