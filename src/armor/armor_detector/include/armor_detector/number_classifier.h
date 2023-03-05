/**
 * @file number_classifier.hpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-02-04
 * 
 */
#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// c++
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// opencv
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

// user
#include "armor_detector/detector.h"

namespace armor_auto_aim
{
class NumberClassifier
{
public:
  NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double threshold);

  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

  void doClassify(std::vector<Armor> & armors);

  double threshold;

private:
  cv::dnn::Net net_;
  std::vector<char> class_names_;
};
}  // namespace armor_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
