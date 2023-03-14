/**
 * @file tracker.h
 * @brief 对预测进行封装，目前使用EKF
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-01
 * 
 */
#ifndef ARMOR_TRACKER__TRACKER_KALMAN_H
#define ARMOR_TRACKER__TRACKER_KALMAN_H
// c++
#include <vector>
#include <string>
#include <deque>

// opencv
#include <opencv2/core.hpp>

// user
#include "armor_tracker/kalman_filter.h"

/**
 * @brief 记录装甲板坐标信息
 */
struct Armor
{
    char id;
    double time_stamp;
    double area;
    std::string key;
    std::vector<cv::Point2f> positions2d;
    cv::Point2f center2d;
    Eigen::Vector3d center3d_cam;
    Eigen::Vector3d center3d_world;
    Eigen::Vector3d euler;
    Eigen::Vector3d predict;
    Eigen::Vector2d angle;
};

class ArmorTrackerKalman{
public:
    enum State {
        LOST,           //丢失
        DETECTING,      //检测中
        TRACKING,       //跟踪中
        TEMP_LOST,      //临时丢失
    } tracker_state;    //跟踪状态

public:
    ArmorTrackerKalman(
        const KalmanFilterMatrices & kf_matrices, double max_match_distance, int tracking_threshold,
        int lost_threshold);
    void init(std::vector<Armor> & src, double timestmp);
    std::unique_ptr<KalmanFilter> kf_;
    Eigen::VectorXd target_state;
    Armor pre_armor;                    //上一次装甲板
    double pre_timestamp;               //上次装甲板时间戳

    char tracking_id;                   //跟踪id
    KalmanFilterMatrices kf_matrices_;
    Eigen::Vector3d tracking_velocity_;
    double max_lost_distance;           //最大距离
    int matched_count_threshold;        //匹配成功次数阈值
    int lost_count_threshold;           //丢失次数阈值

    double max_lost_time;               //最大丢失时间

    int lost_count;                     //丢失次数
    int matched_count;                  //匹配成功次数

    
    Armor suggest_armor;                //建议装甲板
    bool suggest_fire;                  //建议开火

    
    void update(const std::vector<Armor> & armors, double dt_);
};

#endif // ARMOR_TRACKER__TRACKER_KALMAN_H