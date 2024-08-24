/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros_msgs/srv/mode_switch_service.hpp"
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

void Reset_InitPose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
void handle_service(
  const cartographer_ros_msgs::srv::ModeSwitchService::Request::SharedPtr request,
  cartographer_ros_msgs::srv::ModeSwitchService::Response::SharedPtr response);

std::shared_ptr<cartographer_ros::Node> node_handle;
std::shared_ptr<cartographer_ros::TrajectoryOptions> trajectory_options_handle;
// /opt/ros/humble/include/geometry_msgs/geometry_msgs/msg/pose_with_covariance_stamped.hpp
//修改结束

/**
 * note: gflags是一套命令行参数解析工具
 * DEFINE_bool在gflags.h中定义
 * gflags主要支持的参数类型包括bool, int32, int64, uint64, double, string等
 * 定义参数通过DEFINE_type宏实现, 该宏的三个参数含义分别为命令行参数名, 参数默认值, 以及参数的帮助信息
 * 当参数被定义后, 通过FLAGS_name就可访问到对应的参数
 */
// collect_metrics ：激活运行时度量的集合.如果激活, 可以通过ROS服务访问度量
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(configuration_localization_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

enum class CartoMode {
  PureLocalization,
  SLAM,
  CLOSEING,
  UNDEFINED
 };
//默认纯定位模式
CartoMode statue_flag = CartoMode::PureLocalization;

namespace cartographer_ros {
namespace {

//纯定位
void Run() {
  rclcpp::Node::SharedPtr cartographer_node = rclcpp::Node::make_shared("cartographer_node");
  constexpr double kTfBufferCacheTimeInSeconds = 10.;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(),
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
        cartographer_node);

  // 开启监听tf的独立线程
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  NodeOptions node_options;
  TrajectoryOptions trajectory_options;

  // c++11: std::tie()函数可以将变量连接到一个给定的tuple上,生成一个元素类型全是引用的tuple

  // 根据Lua配置文件中的内容, 为node_options, trajectory_options 赋值
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  // MapBuilder类是完整的SLAM算法类
  // 包含前端(TrajectoryBuilders,scan to submap) 与 后端(用于查找回环的PoseGraph) 
  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  // Node类的初始化, 将ROS的topic传入SLAM, 也就是MapBuilder
  auto node = std::make_shared<cartographer_ros::Node>(
    node_options, std::move(map_builder), tf_buffer, cartographer_node,
    FLAGS_collect_metrics);

  // add 
  trajectory_options_handle = std::make_shared<TrajectoryOptions>(trajectory_options);  // 修改
  node_handle = (node);  //修改
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initPose_sub =
  cartographer_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
	"initialpose", rclcpp::SystemDefaultsQoS(),
	std::bind(&Reset_InitPose_callback, std::placeholders::_1));

  ::rclcpp::Service<cartographer_ros_msgs::srv::ModeSwitchService>::SharedPtr server_ =
   cartographer_node->create_service<cartographer_ros_msgs::srv::ModeSwitchService>(
  "switch_carto_mode",
  std::bind(&handle_service, std::placeholders::_1, std::placeholders::_2));

  // 如果加载了pbstream文件, 就执行这个函数
  if (!FLAGS_load_state_filename.empty()) {
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  // 使用默认topic 开始轨迹
  if (FLAGS_start_trajectory_with_default_topics) {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }
  
  rclcpp::spin(cartographer_node);

  // 结束所有处于活动状态的轨迹
  node->FinishAllTrajectories();

  // 当所有的轨迹结束时, 再执行一次全局优化
  node->RunFinalOptimization();

  // 如果save_state_filename非空, 就保存pbstream文件
  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}


void RunSLAM() {
  rclcpp::Node::SharedPtr cartographer_node = rclcpp::Node::make_shared("cartographer_node");
  constexpr double kTfBufferCacheTimeInSeconds = 10.;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(),
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
        cartographer_node);

  // 开启监听tf的独立线程
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  NodeOptions node_options;
  TrajectoryOptions trajectory_options;

  // c++11: std::tie()函数可以将变量连接到一个给定的tuple上,生成一个元素类型全是引用的tuple

  // 根据Lua配置文件中的内容, 为node_options, trajectory_options 赋值
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  // MapBuilder类是完整的SLAM算法类
  // 包含前端(TrajectoryBuilders,scan to submap) 与 后端(用于查找回环的PoseGraph) 
  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  // Node类的初始化, 将ROS的topic传入SLAM, 也就是MapBuilder
  auto node = std::make_shared<cartographer_ros::Node>(
    node_options, std::move(map_builder), tf_buffer, cartographer_node,
    FLAGS_collect_metrics);

  // add 
  trajectory_options_handle = std::make_shared<TrajectoryOptions>(trajectory_options);  // 修改
  node_handle = (node);  //修改
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initPose_sub =
  cartographer_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
	"initialpose", rclcpp::SystemDefaultsQoS(),
	std::bind(&Reset_InitPose_callback, std::placeholders::_1));

  ::rclcpp::Service<cartographer_ros_msgs::srv::ModeSwitchService>::SharedPtr server_ =
   cartographer_node->create_service<cartographer_ros_msgs::srv::ModeSwitchService>(
  "switch_carto_mode",
  std::bind(&handle_service, std::placeholders::_1, std::placeholders::_2));

  // 使用默认topic 开始轨迹
  if (FLAGS_start_trajectory_with_default_topics) {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }
  
  rclcpp::spin(cartographer_node);

  // 结束所有处于活动状态的轨迹
  node->FinishAllTrajectories();

  // 当所有的轨迹结束时, 再执行一次全局优化
  node->RunFinalOptimization();

  // 如果save_state_filename非空, 就保存pbstream文件
  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}
}  // namespace
}  // namespace cartographer_ros

void Reset_InitPose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  // 关闭当前运行的Trajectories
 
  // 给轨迹设置起点 msg->pose.pose
  // start trajectory with initial pose
  // *trajectory_options_handle->trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose()
    // = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(msg->pose.pose));
  cartographer::transform::Rigid2d best_pose_estimate;
   float best_score;
   node_handle->relocate(&best_pose_estimate,&best_score);

    *trajectory_options_handle->trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose()
    = cartographer::transform::ToProto(cartographer::transform::Rigid3d({best_pose_estimate.translation().x(), best_pose_estimate.translation().y(), 0},
                 cartographer::transform::RollPitchYaw(0,0,best_pose_estimate.rotation().angle())));
  node_handle->FinishAllTrajectories();
  // 重新开启Trajectory
  if (FLAGS_start_trajectory_with_default_topics) 
  {
    node_handle->StartTrajectoryWithDefaultTopics(*trajectory_options_handle);
  }
}



void handle_service(
  const cartographer_ros_msgs::srv::ModeSwitchService::Request::SharedPtr request,
  cartographer_ros_msgs::srv::ModeSwitchService::Response::SharedPtr response)
{

  LOG(INFO)<<"Received request: "<<request->request_string.c_str();
	if(request->request_string == "localization")
  {
    if(statue_flag == CartoMode::SLAM)
    {
      node_handle->FinishAllTrajectories();
      ::rclcpp::shutdown();
       sleep(1);
    }
    statue_flag = CartoMode::PureLocalization;
  }
  else if(request->request_string == "slam")
  {
    if(statue_flag == CartoMode::PureLocalization)
    { 
       ::rclcpp::shutdown();
      sleep(1);

    }
    statue_flag = CartoMode::SLAM;
  }
	if(request->request_string == "shutdown")
	{
		// ROS_INFO("shutdown\n");
     if(statue_flag == CartoMode::SLAM)
    {
      node_handle->FinishAllTrajectories();
    }
    ::rclcpp::shutdown();
    statue_flag = CartoMode::CLOSEING;
	}

    // 这里可以添加你自己的逻辑处理请求字符串
    response->response_string = "You said: " + request->request_string;
}

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  google::InitGoogleLogging(argv[0]);
  // 开始运行cartographer_ros
  while (statue_flag != CartoMode::CLOSEING)
  {
      rclcpp::init(argc, argv);
  /**
   * @brief glog里提供的CHECK系列的宏, 检测某个表达式是否为真
   * 检测expression如果不为真, 则打印后面的description和栈上的信息
   * 然后退出程序, 出错后的处理过程和FATAL比较像.
   */
  google::AllowCommandLineReparsing();

  // 使用gflags进行参数的初始化. 其中第三个参数为remove_flag
  // 如果为true, gflags会移除parse过的参数, 否则gflags就会保留这些参数, 但可能会对参数顺序进行调整.
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_directory.empty())
  << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
  << "-configuration_basename is missing.";

      // 使用ROS_INFO进行glog消息的输出
      cartographer_ros::ScopedRosLogSink ros_log_sink;
      std::cout<<"Cycle"<<std::endl;
       switch (statue_flag)
       {
       case CartoMode::PureLocalization:
        cartographer_ros::Run();
        std::cout<<"PureLocalization end"<<std::endl;
        break;
       case CartoMode::SLAM:
        cartographer_ros::RunSLAM();
        std::cout<<"SLAM end"<<std::endl;
        break;
       default:
        break;
       }

  }
   // 结束ROS相关的线程, 网络等
return 0 ;
}
