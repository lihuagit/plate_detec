/**
 * @file detector.h
 * @brief 
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
class Detector
{
public:
	struct LightParams
	{
		// 宽高比范围width / height
		double min_ratio;
		double max_ratio;
		// 最大倾斜角度vertical angle
		double max_angle;
	};
	struct ArmorParams
	{
		double min_light_length_ratio;		// 灯条长度最小比例
		double min_small_light_distance_ratio;	// 小装甲板灯条长度与距离最小比例
		double max_small_light_distance_ratio;	// 小装甲板灯条长度与距离最大比例
		double min_large_light_distance_ratio;	// 大装甲板灯条长度与距离最小比例
		double max_large_light_distance_ratio;	// 大装甲板灯条长度与距离最大比例
		double max_angle;					// 装甲板最大倾斜角度
	};
	Detector(
		const int & init_min_l, const int & init_color, const LightParams & init_l,
		const ArmorParams & init_a, const bool & init_isDebug = false);
	int min_lightness;			// 最小亮度
	int detect_color;			// 检测颜色
	LightParams light_params;
	ArmorParams armor_params;
	bool isDebug;					// 是否调试

	// debug img
	
	// Debug msgs
	armor_interfaces::msg::DebugLights debug_lights;
	armor_interfaces::msg::DebugArmors debug_armors;
	// image_transport::Publisher bin_img_pub_;
	cv::Mat preprocess_img;
	cv::Mat lights_img;
	cv::Mat armors_img;

	// 检测装甲板
	void detectArmor(const cv::Mat & src, std::vector<Armor> & armors);
	// 检测灯条
	void detectLights(const cv::Mat & src_, std::vector<Light> & lights);
	// 匹配灯条,返回装甲板
	void matchLights(const std::vector<Light> & lights, std::vector<Armor> & armors);
	
	~Detector() = default;

private:
	// 预处理
	void preprocess(const cv::Mat & src_img, cv::Mat & dst);
	// 判断是否是灯条
	bool isLight(const Light & light);
	// 判断是否是装甲板
	bool isArmor(Armor & armor);
	// 判断灯条之间是否有其他灯条
	bool containLight(
		const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
};

} // namespace armor_auto_aim

#endif // !ARMOR_DETECTOR__DETECTOR_H
