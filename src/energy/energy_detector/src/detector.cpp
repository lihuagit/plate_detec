/**
 * @file detector.cpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-27
 * 
 */

#include "energy_detector/detector.h"

namespace energy_auto_aim
{
Detector::Detector()
{
    // 初始化模型
    buff_detector_.initModel("/home/ubuntu/lihua/code/ros/hbut_-rm_-vision_2023/src/energy/energy_detector/model/buff.xml");
}

Detector::~Detector()
{
}

bool Detector::initModel(const std::string & model_path)
{
    return buff_detector_.initModel(model_path);
}

void Detector::detect(cv::Mat & img, std::vector<BuffObject> & results)
{
    buff_detector_.detect(img, results);
}

void Detector::drawResult(cv::Mat & img, const std::vector<BuffObject> & results)
{
    for (auto & obj : results)
    {
        cv::rectangle(img, obj.rect, cv::Scalar(0, 255, 0), 2);
        cv::circle(img, obj.apex[0], 2, cv::Scalar(0, 0, 255), 2);
        cv::circle(img, obj.apex[1], 2, cv::Scalar(0, 0, 255), 3);
        cv::circle(img, obj.apex[2], 2, cv::Scalar(0, 0, 255), 4);
        cv::circle(img, obj.apex[3], 2, cv::Scalar(0, 0, 255), 5);
        cv::circle(img, obj.apex[4], 2, cv::Scalar(0, 0, 255), 6);
    }
}

} // namespace energy_auto_aim