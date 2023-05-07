cmake_minimum_required(VERSION 3.10)
project(trt_armor_detector)

#TensorRT
include(FindTensorRT.cmake)

# 包含setup.cmake文件
# include(${CMAKE_CURRENT_SOURCE_DIR}/FindTensorRT.cmake)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find dependencies ##

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Build ##

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
  #TensorRT
  ${TensorRT_SOURCE}
)

##################TensorRT##########################
include_directories(include)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-O3") # 设置优化选项
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--compiler-options;-fPIC") # 设置位置独立代码选项

cuda_add_library(cuda_lib ${CMAKE_CURRENT_SOURCE_DIR}/src/postprocess.cu ${CMAKE_CURRENT_SOURCE_DIR}/src/preprocess.cu)

target_link_libraries(${PROJECT_NAME} ${TensorRT_LIBS} nvonnxparser cuda cuda_lib)

###################TensorRT##########################

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN armor_auto_aim::TRTDetectorNode
  EXECUTABLE ${PROJECT_NAME}_node
)
## Install ##
ament_auto_package(
  INSTALL_TO_SHARE
  model
)
