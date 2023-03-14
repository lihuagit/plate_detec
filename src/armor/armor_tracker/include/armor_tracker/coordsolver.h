/**
 * @file coordsolver.h
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-02-14
 * 
 */

#ifndef ARMOR_TRACKER__COORDSOVER_H
#define ARMOR_TRACKER__COORDSOVER_H

#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

#include "armor_tracker/general.h"

struct PnPInfo
{
    // 装甲板相机坐标
    Eigen::Vector3d armor_cam;

    // 装甲板世界坐标
    Eigen::Vector3d armor_world;

    // R标相机坐标
    Eigen::Vector3d R_cam;

    // R标世界坐标
    Eigen::Vector3d R_world;

    // pnp解算转化为欧拉角
    Eigen::Vector3d euler;
};

class CoordSolver
{
public:
    // CoordSolver();
    // ~CoordSolver();
    
    bool loadParam(string coord_path,string param_name);

    double dynamicCalcPitchOffset(Eigen::Vector3d &xyz);
    
    PnPInfo pnp(const std::vector<Point2f> &points_pic, const Eigen::Matrix3d &rmat_imu, int method);
    
    Eigen::Vector3d camToWorld(const Eigen::Vector3d &point_camera,const Eigen::Matrix3d &rmat);
    Eigen::Vector3d worldToCam(const Eigen::Vector3d &point_world,const Eigen::Matrix3d &rmat);

    Eigen::Vector3d staticCoordOffset(Eigen::Vector3d &xyz);
    Eigen::Vector2d staticAngleOffset(Eigen::Vector2d &angle);
    Eigen::Vector2d getAngle(Eigen::Vector3d &xyz_cam, Eigen::Matrix3d &rmat);

    double calcYaw(Eigen::Vector3d &xyz);
    double calcPitch(Eigen::Vector3d &xyz);
    Eigen::Vector2d calcYawPitch(Eigen::Vector3d &xyz);

    cv::Point2f reproject(Eigen::Vector3d &xyz);
    
    Mat intrinsic_cpy;
private:
    // 使用迭代法求解pitch补偿的最大迭代次数
    int max_iter;
    // 停止迭代的最小误差(单位m)
    float stop_error;
    // 龙格库塔法求解落点的迭代次数
    int R_K_iter;

    // 相机内参矩阵
    Mat intrinsic;
    // 相机畸变矩阵
    Mat dis_coeff;
    // x,y,z坐标偏移量(相机至枪管),单位m
    Eigen::Vector3d xyz_offset;
    // 陀螺仪至云台转轴中心平移向量 R_g = R_i - T_iw
    Eigen::Vector3d t_iw;
    // yaw,pitch角度偏移量,单位度
    Eigen::Vector2d angle_offset;
    // 相机到陀螺仪的变换矩阵 R_i = T_ic * R_c
    Eigen::Matrix4d transform_ic;
    // 陀螺仪到相机的变换矩阵 R_c = T_ci * R_i
    Eigen::Matrix4d transform_ci;

    YAML::Node param_node;

    const int armor_type_wh_thres = 7;      //大小装甲板长宽比阈值
    const int bullet_speed = 28;            //TODO:弹速可变
    // const int bullet_speed = 16;            //TODO:弹速可变
    const double k = 0.0389;                //25°C,1atm,小弹丸
    // const double k = 0.0111;                //25°C,1atm,大弹丸
    const double g = 9.801;
};


#endif // !ARMOR_TRACKER__COORDSOVER_H