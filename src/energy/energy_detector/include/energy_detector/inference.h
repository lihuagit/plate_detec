/**
 * @file inference.h
 * @brief 能量机关推理类  -- 来自沈航
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-27
 * 
 */
// c++
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

// opencv
#include <opencv2/opencv.hpp>
#include <inference_engine.hpp>

// other
#include <Eigen/Core>
#include <fftw3.h>

// user
#include "energy_detector/general.h"

using namespace std;
using namespace cv;
using namespace InferenceEngine;

struct BuffObject
{
    Point2f apex[5];
    cv::Rect_<float> rect;
    int cls;
    int color;
    float prob;
    std::vector<cv::Point2f> pts;
};

class BuffDetector
{

private:
    Core ie;
    CNNNetwork network;                // 网络
    ExecutableNetwork executable_network;       // 可执行网络
    InferRequest infer_request;      // 推理请求
    MemoryBlob::CPtr moutput;
    string input_name;
    string output_name;
    
    Eigen::Matrix<float,3,3> transfrom_matrix;

public:
    BuffDetector();
    ~BuffDetector();

    bool detect(Mat &src,vector<BuffObject>& objects);
    bool initModel(string path);

};
