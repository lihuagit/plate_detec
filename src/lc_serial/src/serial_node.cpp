/**
 * @file serial_node.cpp
 * @brief
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-04
 *
 */

#include "lc_serial/serial_node.h"

SerialDriver::SerialDriver(const rclcpp::NodeOptions& options)
  : Node("lc_serial_driver", options)
  , owned_ctx_{ new IoContext(2) }
  , serial_driver_{ new drivers::serial_driver::SerialDriver(*owned_ctx_) }
{
  RCLCPP_INFO(get_logger(), "Start SerialDriver!");

  getParams();

  // 是否预测、是否pitch增益
  is_track = declare_parameter("is_track", true);
  is_pitch_gain = declare_parameter("is_pitch_gain", true);
  shoot_speed_ = declare_parameter("shoot_speed", 15.0);
  shoot_delay_ = declare_parameter("shoot_delay", 0.4);
  shoot_delay_spin_ = declare_parameter("shoot_delay_spin_", 0.2);


  z_gain = declare_parameter("z_gain", 0.0);
  y_gain = declare_parameter("y_gain", 0.0);
  pitch_gain_ = declare_parameter("pitch_gain_", 1.0);

  // 构造位姿解算器, 定义参数
  int max_iter = declare_parameter("max_iter", 10);
  float stop_error = declare_parameter("stop_error", 0.001);
  int R_K_iter = declare_parameter("R_K_iter", 50);
  coord_solver_ = std::make_unique<CoordSolver>(max_iter, stop_error, R_K_iter);

  // Create Publisher
  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  try
  {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open())
    {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&SerialDriver::receiveData, this);
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error creating lc_serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
  
  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.r = 1.0;
  position_marker_.color.b = 1.0;
  
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/lc_serial/marker", 10);

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      "/processor/target", rclcpp::SensorDataQoS(), std::bind(&SerialDriver::sendData, this, std::placeholders::_1));
}

SerialDriver::~SerialDriver()
{
  if (receive_thread_.joinable())
  {
    receive_thread_.join();
  }
  // SerialDriver sending data: 

  if (serial_driver_->port()->is_open())
  {
    serial_driver_->port()->close();
  }

  if (owned_ctx_)
  {
    owned_ctx_->waitForExit();
  }
}

/**
 * @brief 发送数据
 * @param msg
 */
void SerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  try
  {
    position_marker_.header = msg->header;
    // 未在跟踪状态，不发送数据
    if (msg->tracking == false)
    {
      position_marker_.action = visualization_msgs::msg::Marker::DELETE;
      visualization_msgs::msg::MarkerArray marker_array;
      marker_array.markers.emplace_back(position_marker_);
      marker_pub_->publish(marker_array);
      return;
    }

    // 解算数据
    double yaw = msg->yaw, r1 = msg->radius_1, r2 = msg->radius_2;
    double xc = msg->position.x, yc = msg->position.y, zc = msg->position.z;
    double z2 = msg->z_2;
    double vxc = msg->velocity.x, vyc = msg->velocity.y, vzc = msg->velocity.z;
    double v_yaw = msg->v_yaw;

    z_gain = get_parameter("z_gain").as_double();
    y_gain = get_parameter("y_gain").as_double();

    zc += z_gain;
    z2 += z_gain;
    yc += y_gain;

    // yc-=0.3;
    // 整车坐标
    geometry_msgs::msg::Point point_c;
    point_c.x = xc;
    point_c.y = yc;
    point_c.z = (zc + z2) / 2;

    // 整车速度
    geometry_msgs::msg::Point velocity_c;
    velocity_c.x = vxc;
    velocity_c.y = vyc;
    velocity_c.z = vzc;

    // 整车角速度
    geometry_msgs::msg::Point angular_v_c;
    // 只是为了方便调试，在rviz2中显示，实际上只用到了 z 轴的角速度
    angular_v_c.x = xc;
    angular_v_c.y = yc;
    // angular_v_c.z = v_yaw / M_PI;
    angular_v_c.z = v_yaw;

    // 装甲板坐标，四块装甲板
    std::vector<geometry_msgs::msg::Point> points_a;
    bool use_1 = true;
    for (size_t i = 0; i < 4; i++)
    {
      double tmp_yaw = yaw + i * M_PI_2;
      double r = use_1 ? r1 : r2;
      geometry_msgs::msg::Point point_a;
      point_a.x = xc - r * cos(tmp_yaw);
      point_a.y = yc - r * sin(tmp_yaw);
      point_a.z = use_1 ? zc : z2;
      points_a.push_back(point_a);
      use_1 = !use_1;
    }

    // 子弹飞行速度为 15m/s, 发单延迟为 0.1s
    shoot_speed_ = get_parameter("shoot_speed").as_double();
    shoot_delay_ = get_parameter("shoot_delay").as_double();
    shoot_delay_spin_ = get_parameter("shoot_delay_spin_").as_double();

    // 子弹飞行时间加上发单延迟
    double delay = shoot_delay_ + sqrt(xc*xc + yc*yc + zc*zc) / shoot_speed_;

    // 整车预测坐标
    geometry_msgs::msg::Point point_c_pre;
    point_c_pre.x = point_c.x + velocity_c.x * delay;
    point_c_pre.y = point_c.y + velocity_c.y * delay;
    point_c_pre.z = point_c.z + velocity_c.z * delay;

    // 整车角度预测
    // TODO: 角度预测时间要短些
    delay = shoot_delay_spin_ + sqrt(xc*xc + yc*yc + zc*zc) / shoot_speed_;
    double yaw_pre = yaw + angular_v_c.z * delay;

    // 装甲板预测坐标
    std::vector<geometry_msgs::msg::Point> points_a_pre;
    use_1 = true;
    for (size_t i = 0; i < 4; i++)
    {
      double tmp_yaw = yaw_pre + i * M_PI_2;
      double r = use_1 ? r1 : r2;
      geometry_msgs::msg::Point point_a;
      point_a.x = point_c_pre.x - r * cos(tmp_yaw);
      point_a.y = point_c_pre.y - r * sin(tmp_yaw);
      point_a.z = use_1 ? zc : z2;
      points_a_pre.push_back(point_a);
      use_1 = !use_1;
    }

    // 匹配最优装甲板，面朝摄像头为最优，通过yaw_pre来判断
    // double use_yaw = yaw;
    // if(is_track)SerialDriver sending data: 
    //   use_yaw = yaw_pre;
    // int index = 0;
    // double min = 1000;
    // for (size_t i = 0; i < 4; i++)
    // {
    //   double tmp_yaw = use_yaw + i * M_PI_2;
    //   double tmp = fabs(tmp_yaw);
    //   if (tmp < min)
    //   {
    //     min = tmp;
    //     index = i;
    //   }
    // }

    int index = 0;
    double min = 1000;
    for (size_t i = 0; i < 4; i++)
    {
      double x_p = points_a[i].x;
      double y_p = points_a[i].y;
      if(is_track){
        x_p = points_a_pre[i].x;
        y_p = points_a_pre[i].y;
      }
      double tmp_dis = sqrt(x_p*x_p + y_p*y_p);
      double tmp = fabs(tmp_dis);
      if (tmp < min)
      {
        min = tmp;
        index = i;
      }
    }


    // 对最优装甲板进行位姿解算
    double x, y, z;
    if(is_track){
      x = points_a_pre[index].x;
      y = points_a_pre[index].y;
      z = points_a_pre[index].z;
    }else{
      x = points_a[index].x;
      y = points_a[index].y;
      z = points_a[index].z;
    }

    // 位姿解算
    double send_pitch = atan2(z, sqrt(x * x + y * y));
    double send_yaw = -atan2(y, x);

    // 对抬枪角度进行增益
    if(is_pitch_gain){
      pitch_gain_ = get_parameter("pitch_gain_").as_double();
      coord_solver_->bullet_speed = shoot_speed_;
      Eigen::Vector3d xyz(x, y, z);
      double send_pitch_gain = coord_solver_->dynamicCalcPitchOffset(xyz);
      send_pitch_gain = send_pitch_gain * M_PI / 180.0;
      send_pitch_gain *= xyz.norm() * pitch_gain_;
      RCLCPP_INFO(get_logger(), "send_pitch:%lf", send_pitch);
      RCLCPP_INFO(get_logger(), "send_pitch_gain:%lf", send_pitch_gain);
      RCLCPP_INFO(get_logger(), "x:%lf", x);
      RCLCPP_INFO(get_logger(), "y:%lf", y);
      RCLCPP_INFO(get_logger(), "z:%lf", z);
      send_pitch += send_pitch_gain;
    }

    // 发布marker
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = x;
    position_marker_.pose.position.y = y;
    position_marker_.pose.position.z = z;
  
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.emplace_back(position_marker_);
    marker_pub_->publish(marker_array);

    // 发送数据
    std::vector<uint8_t> data;

    /* 创建一个JSON数据对象(链表头结点) */

    char* str = NULL;

    //"date":[x,y];
    cJSON* cjson_date = cJSON_CreateArray();
    cJSON_AddItemToArray(cjson_date, cJSON_CreateNumber(send_yaw));
    cJSON_AddItemToArray(cjson_date, cJSON_CreateNumber(send_pitch));

    // dat
    cJSON* cjson_dat = cJSON_CreateObject();
    cJSON_AddItemToObject(cjson_dat, "date", cjson_date);
    cJSON_AddStringToObject(cjson_dat, "mode", "visual");

    // send
    cJSON* cjson_send = cJSON_CreateObject();
    cJSON_AddStringToObject(cjson_send, "cmd", "ctr_mode");

    cJSON_AddItemToObject(cjson_send, "dat", cjson_dat);

    str = cJSON_PrintUnformatted(cjson_send);
    int str_len = strlen(str);
    for (int i = 0; i < str_len; i++)
    {
      data.push_back(str[i]);
    }
    data.push_back('\n');

    serial_driver_->port()->send(data);
    RCLCPP_INFO(get_logger(), "SerialDriver sending data: %s", data.data());
    RCLCPP_INFO(get_logger(), "SerialDriver sending data: %d", str_len);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

  /**
   * @brief 不断接收电控数据
   */
  void SerialDriver::receiveData()
  {
    std::vector<uint8_t> data;

    while (rclcpp::ok())
    {
      try
      {
        data.resize(200);
        int rec_len = serial_driver_->port()->receive(data);

        if (rec_len >= 150)
          continue;

        data[rec_len - 1] = '\0';

        double imu_yaw = 0, imu_pitch = 0;

        // 解析json
        cJSON* root = cJSON_Parse((char*)data.data());
        if (!root)
        {
          RCLCPP_INFO(get_logger(), "Error before: [%s]", cJSON_GetErrorPtr());
          continue;
        }
        else
        {
          cJSON* dat = cJSON_GetObjectItem(root, "dat");
          if (!dat)
          {
            RCLCPP_INFO(get_logger(), "Error before: [%s]", cJSON_GetErrorPtr());
            continue;
          }
          else
          {
            imu_yaw = cJSON_GetObjectItem(dat, "imu_yaw")->valuedouble;
            imu_pitch = cJSON_GetObjectItem(dat, "imu_pitch")->valuedouble;
            // RCLCPP_INFO(get_logger(), "imu_yaw: %f", imu_yaw);
            //  RCLCPP_INFO(get_logger(), "imu_pitch: %f", imu_pitch);
          }
        }

        // TODO:收到电控数据
        // RCLCPP_INFO(get_logger(), "SerialDriver receiving data: %s", data.data());
        // RCLCPP_INFO(get_logger(), "SerialDriver receiving len: %d", rec_len);
        if (std::isnan(imu_yaw) || std::isnan(imu_pitch))
          continue;
        try
        {
          sensor_msgs::msg::JointState joint_state;
          joint_state.header.stamp = this->now();
          joint_state.name.push_back("pitch_joint");
          joint_state.name.push_back("yaw_joint");
          joint_state.position.push_back(imu_pitch);
          joint_state.position.push_back(imu_yaw);
          joint_state_pub_->publish(joint_state);
        }
        catch (const std::exception& ex)
        {
          RCLCPP_ERROR(get_logger(), "Error while receiving data: %s", ex.what());
        }
      }
      catch (const std::exception& ex)
      {
        RCLCPP_ERROR(get_logger(), "Error while receiving data: %s", ex.what());
        reopenPort();
      }
    }
  }

  /**
   * @brief 重启串口
   */
  void SerialDriver::reopenPort()
  {
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try
    {
      if (serial_driver_->port()->is_open())
      {
        serial_driver_->port()->close();
      }
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Successfully reopened port");
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
      if (rclcpp::ok())
      {
        rclcpp::sleep_for(std::chrono::seconds(1));
        reopenPort();
      }
    }
  }

  void SerialDriver::getParams()
  {
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate{};
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try
    {
      device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyACM0");
    }
    catch (rclcpp::ParameterTypeException& ex)
    {
      RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
      throw ex;
    }

    try
    {
      baud_rate = declare_parameter<int>("baud_rate", 0);
    }
    catch (rclcpp::ParameterTypeException& ex)
    {
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }

    try
    {
      const auto fc_string = declare_parameter<std::string>("flow_control", "none");

      if (fc_string == "none")
      {
        fc = FlowControl::NONE;
      }
      else if (fc_string == "hardware")
      {
        fc = FlowControl::HARDWARE;
      }
      else if (fc_string == "software")
      {
        fc = FlowControl::SOFTWARE;
      }
      else
      {
        throw std::invalid_argument{ "The flow_control parameter must be one of: none, software, or hardware." };
      }
    }
    catch (rclcpp::ParameterTypeException& ex)
    {
      RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
      throw ex;
    }

    try
    {
      const auto pt_string = declare_parameter<std::string>("parity", "none");

      if (pt_string == "none")
      {
        pt = Parity::NONE;
      }
      else if (pt_string == "odd")
      {
        pt = Parity::ODD;
      }
      else if (pt_string == "even")
      {
        pt = Parity::EVEN;
      }
      else
      {
        throw std::invalid_argument{ "The parity parameter must be one of: none, odd, or even." };
      }
    }
    catch (rclcpp::ParameterTypeException& ex)
    {
      RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
      throw ex;
    }

    try
    {
      const auto sb_string = declare_parameter<std::string>("stop_bits", "1");

      if (sb_string == "1" || sb_string == "1.0")
      {
        sb = StopBits::ONE;
      }
      else if (sb_string == "1.5")
      {
        sb = StopBits::ONE_POINT_FIVE;
      }
      else if (sb_string == "2" || sb_string == "2.0")
      {
        sb = StopBits::TWO;
      }
      else
      {
        throw std::invalid_argument{ "The stop_bits parameter must be one of: 1, 1.5, or 2." };
      }
    }
    catch (rclcpp::ParameterTypeException& ex)
    {
      RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
      throw ex;
    }

    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
  }

#include "rclcpp_components/register_node_macro.hpp"

  // Register the component with class_loader.
  // This acts as a sort of entry point, allowing the component to be discoverable when its library
  // is being loaded into a running process.
  RCLCPP_COMPONENTS_REGISTER_NODE(SerialDriver)