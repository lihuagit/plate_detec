/**
 * @file detector.h
 * @brief 对inference封装
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-27
 * 
 */

#ifndef ENERGY_DETECTOR__DETECTOR_H
#define ENERGY_DETECTOR__DETECTOR_H

// c++

// opencv

// other

// user
#include "energy_detector/inference.h"

namespace energy_auto_aim
{
class Detector
{
public:
    Detector();
    ~Detector();

private:
    BuffDetector buff_detector_;

public:
    // 初始化模型
    bool initModel(const std::string & model_path);

    // 推理
    void detect(cv::Mat & img, std::vector<BuffObject> & results);

    // 可视化
    void drawResult(cv::Mat & img, const std::vector<BuffObject> & results);
};

} // namespace energy_auto_aim

#endif // ENERGY_DETECTOR__DETECTOR_H