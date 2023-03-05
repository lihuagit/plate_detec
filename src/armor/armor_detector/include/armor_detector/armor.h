/**
 * @file armor.hpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-01-30
 * 
 */
#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_


// c++
#include <algorithm>
#include <string>

// opencv
#include <opencv2/core.hpp>

namespace armor_auto_aim
{
enum ArmorType { SMALL = 0, LARGE = 1 };
const int RED = 0, BLUE = 1;

/// @brief 自定义的灯条，继承自cv::RotatedRect
struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }
  bool operator==(const Light & rhs) const
  {
    return (center == rhs.center) && (size == rhs.size) && (angle == rhs.angle);
  }

  int color;   // 灯条的颜色
  cv::Point2f top, bottom;    // 灯条的上下两个端点
  double length;              // 灯条的长度
  double width;               // 灯条的宽度
  float tilt_angle;           // 灯条的倾斜角度
};

/// @brief 装甲板自定义类
struct Armor
{
  Armor() = default;
  Armor(const Light & l1, const Light & l2)
  {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  Light left_light, right_light;  // 装甲板的左右两个灯条
  cv::Point2f center;             // 装甲板的中心点

  cv::Mat number_img;             // 装甲板上的数字图像

  char number;                    // 装甲板上的数字
  // std::string number;
  float similarity;               // 装甲板上的数字和模板的相似度
  float confidence;               // 装甲板的置信度
  std::string classfication_result; // 装甲板的分类结果
  ArmorType armor_type;           // 装甲板的类型
};

}  // namespace armor_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
