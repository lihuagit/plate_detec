/**
 * @file detector.h
 * @brief 装甲板检测类
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-02-03
 * 
 */

#ifndef ARMOR_DETECTOR__DETECTOR_H
#define ARMOR_DETECTOR__DETECTOR_H

// c++
#include <string>
#include <vector>

// opencv
#include <opencv2/core.hpp>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

// user
#include "armor_detector/armor.h"
#include "armor_interfaces/msg/debug_armors.hpp"
#include "armor_interfaces/msg/debug_lights.hpp"

namespace armor_auto_aim
{
/// @brief 装甲板检测类
class Detector
{
public:
	/// @brief 灯条参数
	struct LightParams
	{
		double min_ratio;	// 宽高比范围width / height 最小值
		double max_ratio;	// 宽高比范围width / height 最大值
		double max_angle;	// 最大倾斜角度vertical angle
	};

	/// @brief 装甲板参数
	struct ArmorParams
	{
		double min_light_length_ratio;			// 灯条长度最小比例
		double min_small_light_distance_ratio;	// 小装甲板灯条长度与距离最小比例
		double max_small_light_distance_ratio;	// 小装甲板灯条长度与距离最大比例
		double min_large_light_distance_ratio;	// 大装甲板灯条长度与距离最小比例
		double max_large_light_distance_ratio;	// 大装甲板灯条长度与距离最大比例
		double max_angle;						// 装甲板最大倾斜角度，单位：弧度
	};
	Detector(
		const int & init_min_l, const int & init_color, const LightParams & init_l,
		const ArmorParams & init_a, const bool & init_isDebug = false);
	int min_lightness;			// 预处理时的最小亮度
	int detect_color;			// 检测颜色
	LightParams light_params;	// 灯条参数
	ArmorParams armor_params;	// 装甲板参数

	bool detectArmor(const cv::Mat & src_img, std::vector<Armor> & armors);
	bool detectLights(const cv::Mat & src_img, std::vector<Light> & lights);
	bool matchLights(const std::vector<Light> & lights, std::vector<Armor> & armors);
	~Detector() = default;

	

/////////////////////////////调试参数start////////////////////////////////////
	bool isDebug;				// 是否调试
	armor_interfaces::msg::DebugLights debug_lights;	// 调试灯条参数
	armor_interfaces::msg::DebugArmors debug_armors;	// 调试装甲板参数
	cv::Mat debug_preprocess_img;						// 预处理的图像
/////////////////////////////调试参数end/////////////////////////////////////

private:
	bool preprocess(const cv::Mat & src_img, cv::Mat & dst_img);
	bool isLight(const Light & light);
	bool isArmor(Armor & armor);
	bool containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
};

} // namespace armor_auto_aim

#endif // !ARMOR_DETECTOR__DETECTOR_H
