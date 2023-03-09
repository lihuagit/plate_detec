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

/**
 * @brief 预处理，主要是用腐蚀膨胀消除噪点
 * @param src_img 源图像
 * @param dst_img 预处理后的图像
 * @return true 
 * @return false TODO: 简单的腐蚀膨胀基本不会出错，所以这里不做错误处理，后续可以加上
 */
bool Detector::preprocess(const cv::Mat & src_img, cv::Mat & dst_img)
{
	// 转换为灰度图
	cv::Mat gray_img;
	cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
	// 二值化
	cv::Mat binary_img;
	cv::threshold(gray_img, binary_img, min_lightness, 255, cv::THRESH_BINARY);

	// 膨胀腐蚀
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	// 腐蚀
	cv::erode(binary_img, dst_img, element);
	// 膨胀
	cv::dilate(dst_img, dst_img, element);

	return true;
}

/**
 * @brief 通过LightParams参数，判断是否是灯条
 * @param light 待确定灯条
 * @return true 
 * @return false 
 */
bool Detector::isLight(const Light & light)
{
	bool is_light = false;
	// 宽高比
	double ratio = light.width / light.length;
	// 倾斜角度
	double angle = light.tilt_angle;
	// 判断是否是灯条
	if (ratio > light_params.min_ratio &&
		ratio < light_params.max_ratio &&
		angle < light_params.max_angle)
	{
		is_light = true;
	}
	
	// 调试
	if(isDebug){
		armor_interfaces::msg::DebugLight light_data;
		light_data.center_x = light.center.x;
		light_data.ratio = ratio;
		light_data.angle = light.tilt_angle;
		light_data.is_light = is_light;
		debug_lights.data.emplace_back(light_data);
	}
	
	return is_light;
}

/**
 * @brief 寻找灯条
 * 		  1. 预处理
 * 		  2. 轮廓检测
 * 		  3. 灯条判定
 * @param src_img 待检测图像
 * @param lights 检测完成的灯条
 * @return true 成功检测到灯条
 * @return false 没有检测到灯条
 */
bool Detector::detectLights(const cv::Mat & src_img, std::vector<Light> & lights)
{
	/// 预处理
	cv::Mat binary;
	preprocess(src_img, binary);

	// 调试
	if(isDebug)
		debug_preprocess_img = binary.clone();

	/// 轮廓检测
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	/// 灯条判定
	for (auto & contour : contours)
	{
		// 创建待确定灯条
		Light light(cv::minAreaRect(contour));
		
		// 判断是否是灯条，是则判断颜色并，加入灯条数组
		if (isLight(light))
		{
			// 灯条外接矩形
      		cv::Rect rect = light.boundingRect();

			// 判断灯条是否越界，
			if (0 <= rect.x && 0 <= rect.width &&
				rect.x + rect.width <= src_img.cols &&
				0 <= rect.y && 0 <= rect.height &&
				rect.y + rect.height <= src_img.rows) {
				
				// 通过遍历灯条外接矩形颜色统计判断灯条颜色
				int sum_r = 0, sum_b = 0;
				auto roi = src_img(rect);
				for (int i = 0; i < roi.rows; i++) 
					for (int j = 0; j < roi.cols; j++) 
						if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
							sum_r += roi.at<cv::Vec3b>(i, j)[0];
							sum_b += roi.at<cv::Vec3b>(i, j)[2];
						}
				
				light.color = sum_r > sum_b ? RED : BLUE;
				lights.emplace_back(light);
			}

			
			// // 遍历灯条区域判断颜色
			// // 遍历灯条旋转矩形区域,相比于外接矩形，旋转矩形更能够包含灯条，弃用
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
			// 		color += (src_img.at<cv::Vec3b>(point)[0] - src_img.at<cv::Vec3b>(point)[2]) > 0 ? -1 : 1;
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
	return lights.size() > 0;
}

/**
 * @brief 判断两个灯条中间是否存在灯条，用于消除误检
 * 		  通过两个灯条确定四个点，然后通过这四个点确定一个矩形，判断矩形内是否存在灯条
 * 		  这里高度依赖之前的灯条判定，如果灯条判定不准确，这里也会出现问题
 * 		  TODO: 如上所诉，使用与否，需要进一步测试
 * @param light_1 灯条1
 * @param light_2 灯条2
 * @param lights 灯条数组
 * @return true 灯条中间不存在灯条，符合条件
 * @return false 灯条中间存在灯条，不符合条件
 */
bool Detector::containLight( const Light & light_1, const Light & light_2, const std::vector<Light> & lights ){
	std::vector<cv::Point2f> points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  	cv::Rect bounding_rect = cv::boundingRect(points);

	for (const auto & light : lights) {
		// 跳过两个灯条
		if (light == light_1 || light == light_2) continue;
		// 判断灯条是否在矩形内，端点和中心都进行判定
		if (
			bounding_rect.contains(light.top) ||
			bounding_rect.contains(light.bottom) ||
			bounding_rect.contains(light.center))
		{
			return true;
		}
	}
	return false;
}

/**
 * @brief 通过ArmorParams参数判定是否为装甲板
 * @param armor 待确定装甲板
 * @return true 
 * @return false 
 */
bool Detector::isArmor(Armor & armor)
{
	const Light& light_1 = armor.left_light;
	const Light& light_2 = armor.right_light;
	// 灯条长度比值（短灯条/长灯条）
	float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
																: light_2.length / light_1.length;
	bool light_ratio_ok = light_length_ratio > armor_params.min_light_length_ratio;

	// 灯条长度与两灯条距离比值
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

	bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;
	armor.armor_type = center_distance > armor_params.min_large_light_distance_ratio ? LARGE : SMALL;
	
	// debug
	if(isDebug){
		armor_interfaces::msg::DebugArmor armor_data;
		armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
		armor_data.light_ratio = light_length_ratio;
		armor_data.center_distance = center_distance;
		armor_data.angle = angle;
		armor_data.is_armor = is_armor;
		armor_data.armor_type = (armor.armor_type == LARGE);
		this->debug_armors.data.emplace_back(armor_data);
	}

	return is_armor;
}

/**
 * @brief 	匹配灯条
 * 			判定灯条之间是否有其他灯条
 * 			判定是否符合装甲板参数要求
 * @param lights 待匹配灯条数组
 * @param armors 匹配完成的装甲板数组
 * @return true 成功匹配到装甲板
 * @return false 没有匹配到装甲板
 */
bool Detector::matchLights(
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
			// 待判定装甲板
			Armor armor(lights[i], lights[j]);
			// 判断装甲板
			if (isArmor(armor))
			{
				// 添加装甲板
				armors.emplace_back(armor);
			}
		}
	}
	return armors.size() > 0;
}


/**
 * @brief 检测装甲板，具体操作是检测灯条和匹配灯条，返回装甲板
 * 		  函数直接调用的是detectLights和matchLights
 * @param src 待检测图像
 * @param armors 装甲板数组
 */
bool Detector::detectArmor(const cv::Mat & src_img, std::vector<Armor> & armors)
{
	// 临时灯条数组
	std::vector<Light> lights;
	// 灯条检测
	if( detectLights(src_img, lights) )
		// 匹配灯条
		if( matchLights(lights, armors) )
			return true;

	return false;
}


} // namespace armor_auto_aim