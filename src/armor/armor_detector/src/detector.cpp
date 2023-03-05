/**
 * @file detector.cpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-02-03
 * 
 */

#include "armor_detector/detector.h"

// opnecv
#include <opencv2/opencv.hpp>

namespace armor_auto_aim
{
Detector::Detector(
	const int & init_min_l, const int & init_color, const LightParams & init_l,
	const ArmorParams & init_a, const bool & init_isDebug)
{
	min_lightness = init_min_l;
	detect_color = init_color;
	light_params = init_l;
	armor_params = init_a;
	isDebug = init_isDebug;
}

// 预处理
void Detector::preprocess(const cv::Mat & src, cv::Mat & dst)
{
	// 转换为灰度图
	cv::Mat gray;
	cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
	// 二值化
	cv::Mat binary;
	cv::threshold(gray, binary, min_lightness, 255, cv::THRESH_BINARY);
	// 膨胀
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::dilate(binary, dst, element);
}

// 判断是否是灯条
bool Detector::isLight(const Light & light)
{
	bool is_light = false;
	// 宽高比
	double ratio = light.width / light.length;
	// 倾斜角度
	double angle = light.tilt_angle;
	// 判断是否是灯条
	if (ratio > light_params.min_ratio && ratio < light_params.max_ratio &&
		angle < light_params.max_angle)
	{
		is_light = true;
	}
	armor_interfaces::msg::DebugLight light_data;
	light_data.center_x = light.center.x;
	light_data.ratio = ratio;
	light_data.angle = light.tilt_angle;
	light_data.is_light = is_light;
	this->debug_lights.data.emplace_back(light_data);
	
	return is_light;
}

// 检测灯条
void Detector::detectLights(const cv::Mat & src, std::vector<Light> & lights)
{
	// 预处理
	cv::Mat binary;
	preprocess(src, binary);

	// cv::imshow("src", src);
	// cv::waitKey(1);

	if(isDebug)
		preprocess_img = binary.clone();

	// 轮廓检测
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	// 灯条检测
	for (auto & contour : contours)
	{
		// 灯条
		Light light(cv::minAreaRect(contour));
		
		// 判断是否是灯条
		if (isLight(light))
		{
      		auto rect = light.boundingRect();
			if (  // Avoid assertion failed 
				0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= src.cols && 0 <= rect.y &&
				0 <= rect.height && rect.y + rect.height <= src.rows) {
				int sum_r = 0, sum_b = 0;
				auto roi = src(rect);
				// Iterate through the ROI
				for (int i = 0; i < roi.rows; i++) {
				for (int j = 0; j < roi.cols; j++) {
					if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
					// if point is inside contour
					sum_r += roi.at<cv::Vec3b>(i, j)[0];
					sum_b += roi.at<cv::Vec3b>(i, j)[2];
					}
				}
				}
				// Sum of red pixels > sum of blue pixels ?
				light.color = sum_r > sum_b ? RED : BLUE;
				lights.emplace_back(light);
			}
			
			// // 遍历灯条区域判断颜色
			// for (int i = 0; i < light.boundingRect().height; i++)
			// {
			// 	for (int j = 0; j < light.boundingRect().width; j++)
			// 	{
			// 		// 灯条区域
			// 		// cv::Point2f point = light.center + cv::Point2f(
			// 		// 	(j - light.size.width / 2) * cos(light.angle * CV_PI / 180) -
			// 		// 	(i - light.size.height / 2) * sin(light.angle * CV_PI / 180),
			// 		// 	(j - light.size.width / 2) * sin(light.angle * CV_PI / 180) +
			// 		// 	(i - light.size.height / 2) * cos(light.angle * CV_PI / 180));
			// 		cv::Point2f point(j, i);
			// 		// 统计颜色
			// 		color += (src.at<cv::Vec3b>(point)[0] - src.at<cv::Vec3b>(point)[2]) > 0 ? -1 : 1;
			// 	}
			// }
			// // 判断颜色
			// if (color > 0)
			// {
			// 	light.color = BLUE;
			// }
			// else
			// {
			// 	light.color = RED;
			// }

			// // 添加灯条
			// // lights.push_back(light);
			// lights.emplace_back(light);
		}
	}
}

// 判断灯条之间是否有其他灯条
bool Detector::containLight(
	const Light & light_1, const Light & light_2, const std::vector<Light> & lights){
	std::vector<cv::Point2f> points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  	cv::Rect bounding_rect = cv::boundingRect(points);

	for (const auto & test_light : lights) {
		if (test_light == light_1 || test_light == light_2) continue;
		if (
			bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
			bounding_rect.contains(test_light.center))
		{
			return true;
		}
	}
	return false;
}

// 判断是否是装甲板
bool Detector::isArmor(Armor & armor)
{
	const Light& light_1 = armor.left_light;
	const Light& light_2 = armor.right_light;
	// 灯条长度比值（短灯条/长灯条）
	float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
																: light_2.length / light_1.length;
	bool light_ratio_ok = light_length_ratio > armor_params.min_light_length_ratio;

	// 灯条长度与距离比值
	float avg_light_length = (light_1.length + light_2.length) / 2;
	float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
	bool center_distance_ok = (armor_params.min_small_light_distance_ratio < center_distance &&
								center_distance < armor_params.max_small_light_distance_ratio) ||
								(armor_params.min_large_light_distance_ratio < center_distance &&
								center_distance < armor_params.max_large_light_distance_ratio);

	// 灯条中心点连线与水平线夹角
	cv::Point2f diff = light_1.center - light_2.center;
	float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
	bool angle_ok = angle < armor_params.max_angle;
	// std::cout<<"angle:"<<angle<<std::endl;

	bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;
	armor.armor_type = center_distance > armor_params.min_large_light_distance_ratio ? LARGE : SMALL;
	// Fill in debug information
	armor_interfaces::msg::DebugArmor armor_data;
	armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
	armor_data.light_ratio = light_length_ratio;
	armor_data.center_distance = center_distance;
	armor_data.angle = angle;
	armor_data.is_armor = is_armor;
	armor_data.armor_type = (armor.armor_type == SMALL);
	this->debug_armors.data.emplace_back(armor_data);

	return is_armor;
}


// 匹配灯条,返回装甲板
void Detector::matchLights(
	const std::vector<Light> & lights, std::vector<Armor> & armors)
{
	// 匹配灯条
	long long light_num = lights.size();
	for (int i = 0; i < light_num; i++)
	{
		for (int j = i + 1; j < light_num; j++)
		{
			// 判断颜色
      		if (lights[i].color != detect_color || lights[j].color != detect_color) continue;
			// 判断灯条之间是否有其他灯条
			if (containLight(lights[i], lights[j], lights)) continue;
			// 匹配灯条
			Armor armor(lights[i], lights[j]);
			// 判断装甲板
			if (isArmor(armor))
			{
				// 添加装甲板
				armors.emplace_back(armor);
			}
		}
	}
	// std::cout<<"armors.size()"<<armors.size()<<std::endl;
}

// 检测装甲板
void Detector::detectArmor(const cv::Mat & src, std::vector<Armor> & armors)
{
	// std::cout<<"light start"<<std::endl;
	// 灯条
	std::vector<Light> lights;
	// 灯条检测
	detectLights(src, lights);
	// std::cout<<"armor start"<<std::endl;
	// 匹配灯条
	matchLights(lights, armors);
}


} // namespace armor_auto_aim