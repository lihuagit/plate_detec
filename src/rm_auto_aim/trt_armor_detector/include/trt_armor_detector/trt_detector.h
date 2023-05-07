#ifndef __PLATE_DETECT__
#define __PLATE_DETECT__
// c++
#include <iostream>
#include <fstream>
#include <unordered_map>

// nvidia
#include "NvInfer.h"

// user
#include "trt_armor_detector/logging.h"
#include "trt_armor_detector/utils.hpp"
#include "trt_armor_detector/preprocess.h"
#include "trt_armor_detector/postprocess.h"

#define MAX_IMAGE_INPUT_SIZE_THRESH 5000 * 5000
#define MAX_OBJECTS 1024
#define NUM_BOX_ELEMENT 15
#define NUM_CLASSES 36
#define INPUT_W 640
#define INPUT_H 640

using namespace nvinfer1;

struct affine_matrix
{
    float i2d[6];
    float d2i[6];
};
const int BLUE = 0;
const int RED = 1;
const int NONE = 2;
const int PURPLE = 3;

struct bbox 
{
    float x1,y1,x2,y2;
    float key_points[8];  //关键点4个
    float score;
    int label;
    int color;
    std::string number;
    double ratio;
};

class TRTDetector
{
public:
    TRTDetector();
    void loadTrtModel(const char *trtmodel);
    void detect(cv::Mat &img,std::vector<bbox> &bboxes,float prob_threshold = 0.25f, float nms_threshold = 0.45f);
    ~TRTDetector();

private:
    IRuntime* runtime;
    ICudaEngine* engine;
    IExecutionContext* context;
    float * prob;  //trt输出 
    void *buffers[2]; 
    int output_size ;   //trt输出大小 
    int output_candidates; //640输入是 25200

    const char* input_blob_name; //onnx 输入  名字
    const char* output_blob_name; //onnx 输出 名字

    int input_w = INPUT_W;
    int input_h = INPUT_H;

    cudaStream_t stream;

    uint8_t* img_host;
    uint8_t* img_device;
    float *affine_matrix_host;
    float *affine_matrix_device;
    float *decode_ptr_host;
    float *decode_ptr_device;

    // 装甲板标签map
    std::unordered_map<int, std::string> label_map;
};
#endif