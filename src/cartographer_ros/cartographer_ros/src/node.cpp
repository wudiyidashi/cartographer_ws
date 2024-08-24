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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
// #include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
// #include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
//#include "ros/serialization.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

using ::cartographer::mapping::NodeId;
using ::cartographer::mapping::MapById;
using ::cartographer::mapping::TrajectoryNode;
using ::cartographer::sensor::RangefinderPoint;

namespace {
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.

/**
 * @brief 在node_handle中订阅topic,并与传入的回调函数进行注册
 * 
 * @tparam MessageType 模板参数,消息的数据类型
 * @param[in] handler 函数指针, 接受传入的函数的地址
 * @param[in] trajectory_id 轨迹id
 * @param[in] topic 订阅的topic名字
 * @param[in] node_handle ros的node_handle
 * @param[in] node node类的指针
 * @return ::ros::Subscriber 订阅者
 */
template <typename MessageType>
::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstSharedPtr&),
    const int trajectory_id, const std::string& topic,
    ::rclcpp::Node::SharedPtr node_handle, Node* const node) {
  return node_handle->create_subscription<MessageType>(
      topic, rclcpp::SensorDataQoS(),
      // c++11: lambda表达式
      [node, handler, trajectory_id, topic](const typename MessageType::ConstSharedPtr msg) {
            (node->*handler)(trajectory_id, topic, msg);
          });
}

// 返回轨迹的状态
std::string TrajectoryStateToString(const TrajectoryState trajectory_state) {
  switch (trajectory_state) {
    case TrajectoryState::ACTIVE:
      return "ACTIVE";
    case TrajectoryState::FINISHED:
      return "FINISHED";
    case TrajectoryState::FROZEN:
      return "FROZEN";
    case TrajectoryState::DELETED:
      return "DELETED";
  }
  return "";
}

}  // namespace

/**
 * @brief
 * 声明ROS的一些topic的发布器, 服务的发布器, 以及将时间驱动的函数与定时器进行绑定
 *
 * @param[in] node_options 配置文件的内容
 * @param[in] map_builder SLAM算法的具体实现
 * @param[in] tf_buffer tf
 * @param[in] collect_metrics 是否启用metrics,默认不启用
 */
Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Node::SharedPtr node,
    const bool collect_metrics)
    : node_options_(node_options)
{
  node_ = node;
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_) ;
  map_builder_bridge_.reset(new cartographer_ros::MapBuilderBridge(node_options_, std::move(map_builder), tf_buffer.get()));

  absl::MutexLock lock(&mutex_);
  // if (collect_metrics) {
  //   metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
  //   carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  // }

  // Step: 1 声明需要发布的topic

  // 发布SubmapList
  submap_list_publisher_ =
      node_->create_publisher<::cartographer_ros_msgs::msg::SubmapList>(
          kSubmapListTopic, 10);
  // 发布轨迹
  trajectory_node_list_publisher_ =
      node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kTrajectoryNodeListTopic, 10);
  // 发布landmark_pose
  landmark_poses_list_publisher_ =
      node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kLandmarkPosesListTopic, 10);
  // 发布约束        
  constraint_list_publisher_ =
      node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kConstraintListTopic, 10);
  // 发布tracked_pose, 默认不发布
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_ =
        node_->create_publisher<::geometry_msgs::msg::PoseStamped>(
            kTrackedPoseTopic, 10);
  }

  // Step: 2 处理之后的点云的发布器
  scan_matched_point_cloud_publisher_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        kScanMatchedPointCloudTopic, 10);
  // Step: 3 创建Service
  submap_query_server_ = node_->create_service<cartographer_ros_msgs::srv::SubmapQuery>(
      kSubmapQueryServiceName,
      std::bind(
          &Node::handleSubmapQuery, this, std::placeholders::_1, std::placeholders::_2));
  trajectory_query_server = node_->create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
      kTrajectoryQueryServiceName,
      std::bind(
          &Node::handleTrajectoryQuery, this, std::placeholders::_1, std::placeholders::_2));
  start_trajectory_server_ = node_->create_service<cartographer_ros_msgs::srv::StartTrajectory>(
      kStartTrajectoryServiceName,
      std::bind(
          &Node::handleStartTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  finish_trajectory_server_ = node_->create_service<cartographer_ros_msgs::srv::FinishTrajectory>(
      kFinishTrajectoryServiceName,
      std::bind(
          &Node::handleFinishTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  write_state_server_ = node_->create_service<cartographer_ros_msgs::srv::WriteState>(
      kWriteStateServiceName,
      std::bind(
          &Node::handleWriteState, this, std::placeholders::_1, std::placeholders::_2));
  get_trajectory_states_server_ = node_->create_service<cartographer_ros_msgs::srv::GetTrajectoryStates>(
      kGetTrajectoryStatesServiceName,
      std::bind(
          &Node::handleGetTrajectoryStates, this, std::placeholders::_1, std::placeholders::_2));
  // read_metrics_server_ = node_->create_service<cartographer_ros_msgs::srv::ReadMetrics>(
  //     kReadMetricsServiceName,
  //     std::bind(
  //         &Node::handleReadMetrics, this, std::placeholders::_1, std::placeholders::_2));

  relocate_server_ = node_->create_service<cartographer_ros_msgs::srv::Relocate>(
              "relocate", std::bind(&Node::handleRelocate, this, std::placeholders::_1,std::placeholders::_2));
  
  // Step: 4 进行定时器与函数的绑定, 定时发布数据
  submap_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
    [this]() {
      PublishSubmapList();
    });
  if (node_options_.pose_publish_period_sec > 0) {
    local_trajectory_data_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(int(node_options_.pose_publish_period_sec * 1000)),
      [this]() {
        PublishLocalTrajectoryData();
      });
  }
  trajectory_node_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    [this]() {
      PublishTrajectoryNodeList();
    });
  landmark_pose_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    [this]() {
      PublishLandmarkPosesList();
    });
  constrain_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(kConstraintPublishPeriodSec * 1000)),
    [this]() {
      PublishConstraintList();
    });
}

// 在析构是执行一次全局优化
Node::~Node() { FinishAllTrajectories(); }

/**
 * @brief 获取对应id轨迹的 索引为submap_index 的submap
 *
 * @param[in] request 获取submap的请求
 * @param[out] response 服务的回应
 * @return true: ROS的service只能返回true, 返回false程序会中断
 */
bool Node::handleSubmapQuery(
    const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->HandleSubmapQuery(request, response);
  return true;
}

/**
 * @brief 获取对应id的轨迹
 *
 * @param[in] request
 * @param[out] response
 * @return true: ROS的service只能返回true, 返回false程序会中断
 */
bool Node::handleTrajectoryQuery(
    const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  // 检查对应id的轨迹是否存在, 如果存在判断一下该id轨迹的状态
  response->status = TrajectoryStateToStatus(
      request->trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN} /* valid states */);
  if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response->status.message;
    return true;
  }
  // 获取轨迹
  map_builder_bridge_->HandleTrajectoryQuery(request, response);
  return true;
}

/**
 * @brief 每0.3s发布一次submap list,
 * 这里的submap只有节点的id与当前submap的节点数, 并没有地图数据
 *
 * @param[in] unused_timer_event
 */
void Node::PublishSubmapList() {
  absl::MutexLock lock(&mutex_);
  submap_list_publisher_->publish(map_builder_bridge_->GetSubmapList(node_->now()));
}

/**
 * @brief 新增一个位姿估计器 imu和里程计的融合 预测 前端匹配先验
 * 
 * @param[in] trajectory_id 轨迹id
 * @param[in] options 参数配置
 */
void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
    // 新生成的轨迹的id 不应该在extrapolators_中
  CHECK(extrapolators_.count(trajectory_id) == 0);

    // imu_gravity_time_constant在2d, 3d中都是10
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();

  // c++11: map::emplace() 用于通过在容器中插入新元素来扩展map容器
  // 元素是直接构建的（既不复制也不移动）.仅当键不存在时才进行插入
  // c++11: std::forward_as_tuple tuple的完美转发
  // 该 tuple 在以右值为参数时拥有右值引用数据成员, 否则拥有左值引用数据成员
  // c++11: std::piecewise_construct 分次生成tuple的标志常量
  // 在map::emplace()中使用forward_as_tuple时必须要加piecewise_construct,不加就报错
  // https://www.cnblogs.com/guxuanqing/p/11396511.html

  // 以1ms, 以及重力常数10, 作为参数构造PoseExtrapolator
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

/**
 * @brief 新生成一个传感器数据采样器
 * 
 * @param[in] trajectory_id 轨迹id
 * @param[in] options 参数配置
 */
void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}


/**
 * @brief 每5e-3s发布一次tf与tracked_pose
 *
 * @param[in] timer_event
 */
void Node::PublishLocalTrajectoryData() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_->GetLocalTrajectoryData()) {
    const auto& trajectory_data = entry.second;
        // entry的数据类型为std::unordered_map<int,MapBuilderBridge::LocalTrajectoryData>
    // entry.first 就是轨迹的id, entry.second 就是 LocalTrajectoryData

   // 获取对应轨迹id的extrapolator
    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    // 如果当前状态的时间与extrapolator的lastposetime不等
    if (trajectory_data.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      // 有订阅才发布scan_matched_point_cloud
      if (scan_matched_point_cloud_publisher_->get_subscription_count() > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local
                                .returns.size());
        // 获取local_slam_data的点云数据, 填入到point_cloud中
        for (const cartographer::sensor::RangefinderPoint & point :
             trajectory_data.local_slam_data->range_data_in_local.returns) {
        // 这里的虽然使用的是带时间戳的点云结构, 但是数据点的时间全是0.f
          point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(
              point, 0.f /* time */));
        }

        // 先将点云转换成ROS的格式,再发布scan_matched_point_cloud点云
        scan_matched_point_cloud_publisher_->publish(ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_data.local_slam_data->time),
            node_options_.map_frame,
            // 将雷达坐标系下的点云转换成地图坐标系下的点云
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_data.local_to_map.cast<float>())));
      }
      // 将当前的pose加入到extrapolator中, 更新extrapolator的时间与状态
      extrapolator.AddPose(trajectory_data.local_slam_data->time,
                           trajectory_data.local_slam_data->local_pose);
    }

    geometry_msgs::msg::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    // 使用较新的时间戳
    const ::cartographer::common::Time now = std::max(
        FromRos(node_->now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp =
        node_options_.use_pose_extrapolator
            ? ToRos(now)
            : ToRos(trajectory_data.local_slam_data->time);

    // Suppress publishing if we already published a transform at this time.
    // Due to 2020-07 changes to geometry2, tf buffer will issue warnings for
    // repeated transforms with the same timestamp.
    if (last_published_tf_stamps_.count(entry.first) &&
        last_published_tf_stamps_[entry.first] == stamped_transform.header.stamp)
      continue;
     // 保存当前的时间戳, 以防止对同一时间戳进行重复更新
    last_published_tf_stamps_[entry.first] = stamped_transform.header.stamp;
    // 获取当前位姿在local坐标系下的坐标
    const Rigid3d tracking_to_local_3d =  
        node_options_.use_pose_extrapolator
            ? extrapolator.ExtrapolatePose(now)
            : trajectory_data.local_slam_data->local_pose;
    const Rigid3d tracking_to_local = [&] {
      // 是否将变换投影到平面上
      if (trajectory_data.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(tracking_to_local_3d));
      }
      return tracking_to_local_3d;
    }();
    // 求得当前位姿在map下的坐标
    const Rigid3d tracking_to_map =
        trajectory_data.local_to_map * tracking_to_local;
    // 根据lua配置文件发布tf
    if (trajectory_data.published_to_tracking != nullptr) {
      // 如果需要cartographer提供odom坐标系
        // 则发布 map_frame -> odom -> published_frame 的tf
      if (node_options_.publish_to_tf) {
        if (trajectory_data.trajectory_options.provide_odom_frame) {
          std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms;
          // map_frame -> odom
          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.odom_frame;
          // 将local坐标系作为odom坐标系
          stamped_transform.transform =
              ToGeometryMsgTransform(trajectory_data.local_to_map);
          stamped_transforms.push_back(stamped_transform);
          // odom -> published_frame
          stamped_transform.header.frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          // published_to_tracking 是局部坐标系下的位姿
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_local * (*trajectory_data.published_to_tracking));
          stamped_transforms.push_back(stamped_transform);
          // 发布 map_frame -> odom -> published_frame 的tf
          tf_broadcaster_->sendTransform(stamped_transforms);
        } else {
           // cartographer不需要提供odom坐标系,则发布 map_frame -> published_frame 的tf
          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_map * (*trajectory_data.published_to_tracking));
          // 发布 map_frame -> published_frame 的tf
          tf_broadcaster_->sendTransform(stamped_transform);
          // LOG(INFO)<<"发布 "<<node_options_.map_frame<<" -> "<< trajectory_data.trajectory_options.published_frame<<" 的tf";
        }
      }
       // publish_tracked_pose 默认为false, 默认不发布
      // 如果设置为true, 就发布一个在tracking_frame处的pose
      if (node_options_.publish_tracked_pose) {
        ::geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = node_options_.map_frame;
        pose_msg.header.stamp = stamped_transform.header.stamp;
        pose_msg.pose = ToGeometryMsgPose(tracking_to_map);
        tracked_pose_publisher_->publish(pose_msg);
      }
    }
  }
}

// 每30e-3s发布一次轨迹路径点数据
void Node::PublishTrajectoryNodeList() {
    // 只有存在订阅者的时候才发布轨迹
  if (trajectory_node_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_->publish(
        map_builder_bridge_->GetTrajectoryNodeList(node_->now()));
  }
}

// 每30e-3s发布一次landmark pose 数据
void Node::PublishLandmarkPosesList() {
  if (landmark_poses_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_->publish(
        map_builder_bridge_->GetLandmarkPosesList(node_->now()));
  }
}

// 每0.5s发布一次约束数据
void Node::PublishConstraintList() {
  if (constraint_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_->publish(map_builder_bridge_->GetConstraintList(node_->now()));
  }
}

void Node::PublishPointCloudMap() {
  // // 纯定位时不发布点云地图
  // if (load_state_ || point_cloud_map_publisher_.getNumSubscribers() == 0) {
  //   return;
  // }

  // // 只发布轨迹id 0 的点云地图
  // constexpr int trajectory_id = 0;

  // // 获取优化后的节点位姿与节点的点云数据
  // std::shared_ptr<MapById<NodeId, TrajectoryNode>> trajectory_nodes =
  //     map_builder_bridge_.GetTrajectoryNodes();

  // // 如果个数没变就不进行地图发布
  // size_t trajectory_nodes_size = trajectory_nodes->SizeOfTrajectoryOrZero(trajectory_id);
  // if (last_trajectory_nodes_size_ == trajectory_nodes_size)
  //   return;
  // last_trajectory_nodes_size_ = trajectory_nodes_size;

  // absl::MutexLock lock(&point_cloud_map_mutex_);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_map(new pcl::PointCloud<pcl::PointXYZ>());
  // pcl::PointCloud<pcl::PointXYZ>::Ptr node_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  
  // // 遍历轨迹0的所有优化后的节点
  // auto node_it = trajectory_nodes->BeginOfTrajectory(trajectory_id);
  // auto end_it = trajectory_nodes->EndOfTrajectory(trajectory_id);
  // float cosphi;
  // float sinphi;
  // for (; node_it != end_it; ++node_it) {
  //   auto& trajectory_node = trajectory_nodes->at(node_it->id);
  //   auto& filtered_gravity_aligned_point_cloud = trajectory_node.constant_data->filtered_gravity_aligned_point_cloud;
  //   auto& test = trajectory_node.constant_data->gravity_alignment;
  //   auto& global_pose = trajectory_node.global_pose;
    
  //   if (trajectory_node.constant_data != nullptr) {
  //     node_point_cloud->clear();
  //     node_point_cloud->resize(filtered_gravity_aligned_point_cloud.size());
  //     // 遍历点云的每一个点, 进行坐标变换
  //     for (const RangefinderPoint& point :
  //          filtered_gravity_aligned_point_cloud.points()) {
  //          //std::cout << test.coeffs().z() << std::endl;
           
  //          double x = test.coeffs().x();
  //          double y = test.coeffs().y();
  //          double z = test.coeffs().z();
  //          double w = test.coeffs().w();
  //          cosphi = cos(atan2(2*(z*w + x*y),1 - 2*(y*y + z*z)));
  //          sinphi = sin(atan2(2*(z*w + x*y),1 - 2*(y*y + z*z)));
           
  //       //std::cout << cosphi << " " << sinphi << std::endl;
           
  //          double point_x = point.position.x() * cosphi + point.position.y() * sinphi;// + global_pose.translation().x();//point_x = point.position.x() * cosphi + point_x - point.position.y() * sinphi;
  //          double point_y = -point.position.x() * sinphi + point.position.y() * cosphi;//+ global_pose.translation().y();//point_x = point.position.x() * sinphi + point_x - point.position.y() * sinphi;
  //          //std::cout << point_x << " " << point.position.x() << std::endl;
  //          //point.position.x() = point_x;
  //          //point.position.y() = point_y;
  //          RangefinderPoint range_finder_point_pre;
  //           range_finder_point_pre.position.x() = point_x;
  //           range_finder_point_pre.position.y() = point_y;
  //           range_finder_point_pre.position.z() = 0;
           
  //       RangefinderPoint range_finder_point = global_pose.cast<float>() * range_finder_point_pre;//global_pose.cast<float>() * 
  //       //std::cout << global_pose.rotation().coeffs().x()<< global_pose.rotation().coeffs().y() << global_pose.rotation().coeffs().z() << global_pose.rotation().coeffs().w()  << std::endl;
  //       //std::cout << global_pose.translation().x() << std::endl;
  //       //exit(-1);
        
        
  //       node_point_cloud->push_back(pcl::PointXYZ(
  //           range_finder_point.position.x(), range_finder_point.position.y(),
  //           range_finder_point.position.z()));
   
  //     }
  //     // 将每个节点的点云组合在一起
  //     *point_cloud_map += *node_point_cloud;
  //   }
  // } // end for

  // ros_point_cloud_map_.data.clear();
  // pcl::toROSMsg(*point_cloud_map, ros_point_cloud_map_);
  // ros_point_cloud_map_.header.stamp = ros::Time::now();
  // ros_point_cloud_map_.header.frame_id = node_options_.map_frame;
  // LOG(INFO) << "publish point cloud map";
  // point_cloud_map_publisher_.publish(ros_point_cloud_map_);
}

/**
 * @brief 根据配置文件, 确定所有需要的topic的名字的集合
 *
 * @param[in] options TrajectoryOptions的配置文件
 * @return std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
 */
std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(const TrajectoryOptions& options) const {
 /*
    enum class SensorType {
      RANGE = 0, 雷达、点云
      IMU,
      ODOMETRY,
      FIXED_FRAME_POSE,
      LANDMARK,
      LOCAL_SLAM_RESULT
    };

    struct SensorId {
      SensorType type;  // 传感器的种类
      std::string id;   // topic的名字
    };
  */
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.

  // 如果只有一个传感器, 那订阅的topic就是topic
  // 如果是多个传感器, 那订阅的topic就是topic_1,topic_2, 依次类推
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
         LOG(INFO) << "laser_scan Topic name [" << topic << "].";
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  // 3d slam必须有imu, 2d可有可无, imu的topic的个数只能有一个
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
  }
  // Odometry is optional.
  // 里程计可有可无, topic的个数只能有一个
  if (options.use_odometry) {
         LOG(INFO) << "laser_scan Topic name [" << kOdometryTopic << "].";
    expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
  }
  // NavSatFix is optional.
  // gps可有可无, topic的个数只能有一个
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
  }
  // Landmark is optional.
    // Landmark可有可无, topic的个数只能有一个
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  // 返回传感器的topic名字
  return expected_topics;
}
/**
 * @brief 添加一个新的轨迹
 *
 * @param[in] options 轨迹的参数配置
 * @return int 新生成的轨迹的id
 */
int Node::AddTrajectory(const TrajectoryOptions& options) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options);

  // 调用map_builder_bridge的AddTrajectory, 添加一个轨迹
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  // 新增一个位姿估计器
  AddExtrapolator(trajectory_id, options);
   // 新生成一个传感器数据采样器
  AddSensorSamplers(trajectory_id, options);
    // 订阅话题与注册回调函数
  LaunchSubscribers(options, trajectory_id);

  // 创建了一个3s执行一次的定时器,由于oneshot=true, 所以只执行一次
  // 检查设置的topic名字是否在ros中存在, 不存在则报错
  maybe_warn_about_topic_mismatch_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(kTopicMismatchCheckDelaySec * 1000)),
    [this]() {
      MaybeWarnAboutTopicMismatch();
    });
  // 将topic名字保存下来,用于之后的新建轨迹时检查topic名字是否重复
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

/**
 * @brief 订阅话题与注册回调函数
 * 
 * @param[in] options 配置参数
 * @param[in] trajectory_id 轨迹id  
 */
void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const int trajectory_id) {
  // laser_scan 的订阅与注册回调函数, 多个laser_scan 的topic 共用同一个回调函数                             
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
        LOG(INFO) << "laser_scan Topic name [" << topic << "].";
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, node_, this),
         topic});
  }

  // multi_echo_laser_scans的订阅与注册回调函数
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
        LOG(INFO) << " multi_echo_laser_scans Topic name [" << topic << "].";
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic, node_, this),
         topic});
  }

  // point_clouds 的订阅与注册回调函数
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
        LOG(INFO) << "point_clouds Topic name [" << topic << "].";
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic, node_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  // imu 的订阅与注册回调函数,只有一个imu的topic
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
      LOG(INFO) << "Imu Topic name [" << kImuTopic << "].";
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, kImuTopic,
                                                node_, this),
         kImuTopic});
  }
  // odometry 的订阅与注册回调函数,只有一个odometry的topic
  if (options.use_odometry) {
      LOG(INFO) << "Odom Topic name [" << kOdometryTopic << "].";
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::msg::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, kOdometryTopic,
                                                  node_, this),
         kOdometryTopic});
  }
  // gps 的订阅与注册回调函数,只有一个gps的topic
  if (options.use_nav_sat) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, kNavSatFixTopic,
             node_, this),
         kNavSatFixTopic});
  }
  // landmarks 的订阅与注册回调函数,只有一个landmarks的topic
  if (options.use_landmarks) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::msg::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, kLandmarkTopic,
             node_, this),
         kLandmarkTopic});
  }
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

cartographer_ros_msgs::msg::StatusResponse Node::TrajectoryStateToStatus(
    const int trajectory_id, const std::set<TrajectoryState>& valid_states) {
  const auto trajectory_states = map_builder_bridge_->GetTrajectoryStates();
  cartographer_ros_msgs::msg::StatusResponse status_response;

  const auto it = trajectory_states.find(trajectory_id);
  if (it == trajectory_states.end()) {
    status_response.message = "Trajectory " + std::to_string(trajectory_id) + " doesn't exist.";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    return status_response;
  }

  status_response.message = "Trajectory " + std::to_string(trajectory_id) + " is in '" +
    TrajectoryStateToString(it->second) + "' state.";
  status_response.code =
      valid_states.count(it->second)
          ? cartographer_ros_msgs::msg::StatusCode::OK
          : cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  return status_response;
}

cartographer_ros_msgs::msg::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::msg::StatusResponse status_response;
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    status_response.message =
        "Trajectory " + std::to_string(trajectory_id) + " already pending to finish.";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // First, check if we can actually finish the trajectory.
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    LOG(ERROR) << "Can't finish trajectory: " << status_response.message;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
      entry.subscriber.reset();
      subscribed_topics_.erase(entry.topic);
      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }
  map_builder_bridge_->FinishTrajectory(trajectory_id);
  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  status_response.message =
      "Finished trajectory " + std::to_string(trajectory_id) + ".";
  status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
  return status_response;
}

/**
 * @brief 通过服务来开始一条新的轨迹
 *
 * @param[in] request
 * 配置文件的目录与名字, 是否使用初始位姿, 初始位姿以及其是相对于哪条轨迹的id, 
 * @param[out] response 返回轨迹的状态与id
 * @return true: ROS的service只能返回true, 返回false程序会中断
 */
bool Node::handleStartTrajectory(
    const cartographer_ros_msgs::srv::StartTrajectory::Request::SharedPtr request,
    cartographer_ros_msgs::srv::StartTrajectory::Response::SharedPtr response) {
  TrajectoryOptions trajectory_options;
   // 获取配置文件内容
  std::tie(std::ignore, trajectory_options) = LoadOptions(
      request->configuration_directory, request->configuration_basename);

  // 如果给定了一个初始位姿
  if (request->use_initial_pose) {
    const auto pose = ToRigid3d(request->initial_pose);
    if (!pose.IsValid()) {
      response->status.message =
          "Invalid pose argument. Orientation quaternion must be normalized.";
      LOG(ERROR) << response->status.message;
      response->status.code =
          cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
      return true;
    }

    // Check if the requested trajectory for the relative initial pose exists.
    // 检查 initial_pose 对应的轨迹id是否存在
    response->status = TrajectoryStateToStatus(
        request->relative_to_trajectory_id,
        {TrajectoryState::ACTIVE, TrajectoryState::FROZEN,
         TrajectoryState::FINISHED} /* valid states */);
    if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
      LOG(ERROR) << "Can't start a trajectory with initial pose: "
                 << response->status.message;
      return true;
    }

    ::cartographer::mapping::proto::InitialTrajectoryPose
        initial_trajectory_pose;
    initial_trajectory_pose.set_to_trajectory_id(
        request->relative_to_trajectory_id);
    // 将pose转成proto格式,放进initial_trajectory_pose
    *initial_trajectory_pose.mutable_relative_pose() =
        cartographer::transform::ToProto(pose);
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
        ::cartographer_ros::FromRos(rclcpp::Time(0))));

      // 将初始位姿信息加入到trajectory_options中
    *trajectory_options.trajectory_builder_options
         .mutable_initial_trajectory_pose() = initial_trajectory_pose;
  }

  // 检查TrajectoryOptions是否存在2d或者3d轨迹的配置信息
  if (!ValidateTrajectoryOptions(trajectory_options)) {
    response->status.message = "Invalid trajectory options.";
    LOG(ERROR) << response->status.message;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  // 检查topic名字是否被其他轨迹使用
  } else if (!ValidateTopicNames(trajectory_options)) {
    response->status.message = "Topics are already used by another trajectory.";
    LOG(ERROR) << response->status.message;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else {
    // 检查通过, 添加一个新的轨迹
    response->status.message = "Success.";
    response->trajectory_id = AddTrajectory(trajectory_options);
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  }
  return true;
}

// 使用默认topic名字开始一条轨迹，也就是开始slam
void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
    // 检查TrajectoryOptions是否存在2d或者3d轨迹的配置信息
  CHECK(ValidateTrajectoryOptions(options));
    // 添加一条轨迹
  AddTrajectory(options);
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id : ComputeExpectedSensorIds(bags_options.at(i))) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  return trajectory_id;
}

bool Node::handleGetTrajectoryStates(
    const cartographer_ros_msgs::srv::GetTrajectoryStates::Request::SharedPtr ,
    cartographer_ros_msgs::srv::GetTrajectoryStates::Response::SharedPtr response) {

  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
  absl::MutexLock lock(&mutex_);
  response->status.code = ::cartographer_ros_msgs::msg::StatusCode::OK;
  response->trajectory_states.header.stamp = node_->now();
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
    response->trajectory_states.trajectory_id.push_back(entry.first);
    switch (entry.second) {
      case TrajectoryState::ACTIVE:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::DELETED);
        break;
    }
  }
  return true;
}

bool Node::handleFinishTrajectory(
    const cartographer_ros_msgs::srv::FinishTrajectory::Request::SharedPtr request,
    cartographer_ros_msgs::srv::FinishTrajectory::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->status = FinishTrajectoryUnderLock(request->trajectory_id);
  return true;
}

bool Node::relocate(cartographer::transform::Rigid2d* best_pose_estimate, float* best_score)
{
  // cartographer::transform::Rigid2d best_pose_estimate;
  //   float best_score;
      absl::MutexLock lock(&mutex_);
  LOG(INFO) << "Relocate service SIZE： "<<map_builder_bridge_->GetLocalTrajectoryData().size();
  //Trajectory 条数大于1 

  for (const auto& entry : map_builder_bridge_->GetLocalTrajectoryData()) {
    const auto& trajectory_data = entry.second;
  //auto& trajectory_data = map_builder_bridge_->GetLocalTrajectoryData()[1];
 
      LOG(ERROR)<< "loacl_point_clout.size():"<<trajectory_data.local_slam_data->range_data_in_local.returns.size();
      // 实现重定位逻辑...
      bool trajectory_states = map_builder_bridge_->Relocate(
          0.5,  // 最高0.7
          trajectory_data.local_slam_data->range_data_in_local.returns, best_pose_estimate, best_score);
  LOG(INFO) <<"relocate success. best_score: "+std::to_string(*best_score)+ " best_pose_estimate: "+
        best_pose_estimate->DebugString();
      LOG(INFO) << "best score "<<best_score;
  }
  return true;
}
bool Node::handleRelocate(const std::shared_ptr<cartographer_ros_msgs::srv::Relocate::Request> request,
                    const std::shared_ptr<cartographer_ros_msgs::srv::Relocate::Response> response)
{   
     cartographer::transform::Rigid2d best_pose_estimate;
      float best_score;
      absl::MutexLock lock(&mutex_);
  LOG(INFO) << "Relocate service SIZE： "<<map_builder_bridge_->GetLocalTrajectoryData().size();
  //Trajectory 条数大于1 

  for (const auto& entry : map_builder_bridge_->GetLocalTrajectoryData()) {
    const auto& trajectory_data = entry.second;
  //auto& trajectory_data = map_builder_bridge_->GetLocalTrajectoryData()[1];
 
      LOG(ERROR)<< "loacl_point_clout.size():"<<trajectory_data.local_slam_data->range_data_in_local.returns.size();
      // 实现重定位逻辑...
      bool trajectory_states = map_builder_bridge_->Relocate(
          0.5,  // 最高0.7
          trajectory_data.local_slam_data->range_data_in_local.returns, &best_pose_estimate, &best_score);
      cartographer_ros_msgs::msg::StatusResponse status_response;
      LOG(INFO) << "best score "<<best_score;

      //success 
      if (trajectory_states) {
        status_response.message = "relocate success. best_score: "+std::to_string(best_score)+ " best_pose_estimate: "+
        best_pose_estimate.DebugString();
        status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
        LOG(INFO) << status_response.message;
        response->status = status_response;
        return trajectory_states;
      }
 
  }
      return false;
}

bool Node::handleWriteState(
    const cartographer_ros_msgs::srv::WriteState::Request::SharedPtr request,
    cartographer_ros_msgs::srv::WriteState::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  if (map_builder_bridge_->SerializeState(request->filename,
                                         request->include_unfinished_submaps)) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message =
        "State written to '" + request->filename + "'.";
  } else {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message =
        "Failed to write '" + request->filename + "'.";
  }
  return true;
}

// bool Node::handleReadMetrics(
//     const cartographer_ros_msgs::srv::ReadMetrics::Request::SharedPtr,
//     cartographer_ros_msgs::srv::ReadMetrics::Response::SharedPtr response) {

//   absl::MutexLock lock(&mutex_);
//   response->timestamp = node_->now();
//   if (!metrics_registry_) {
//     response->status.code = cartographer_ros_msgs::msg::StatusCode::UNAVAILABLE;
//     response->status.message = "Collection of runtime metrics is not activated.";
//     return true;
//   }
//   metrics_registry_->ReadMetrics(response);
//   response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
//   response->status.message = "Successfully read metrics.";
//   return true;
// }

void Node::FinishAllTrajectories() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
    if (entry.second == TrajectoryState::ACTIVE) {
      const int trajectory_id = entry.first;
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::msg::StatusCode::OK);
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  absl::MutexLock lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::msg::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
      const int trajectory_id = entry.first;
      if (entry.second == TrajectoryState::ACTIVE) {
        LOG(WARNING)
            << "Can't run final optimization if there are one or more active "
               "trajectories. Trying to finish trajectory with ID "
            << std::to_string(trajectory_id) << " now.";
        CHECK(FinishTrajectory(trajectory_id))
            << "Failed to finish trajectory with ID "
            << std::to_string(trajectory_id) << ".";
      }
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_->RunFinalOptimization();
}

/**
 * @brief 处理里程计数据,里程计的数据走向有2个
 * 第1个是传入PoseExtrapolator,用于位姿预测
 * 第2个是传入SensorBridge,使用其传感器处理函数进行里程计数据处理
 * 
 * @param[in] trajectory_id 轨迹id
 * @param[in] sensor_id 里程计的topic名字
 * @param[in] msg 里程计的ros格式的数据
 */
void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  // extrapolators_使用里程计数据进行位姿预测
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}
// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}
/**
 * @brief 处理imu数据,imu的数据走向有2个
 * 第1个是传入PoseExtrapolator,用于位姿预测与重力方向的确定
 * 第2个是传入SensorBridge,使用其传感器处理函数进行imu数据处理
 * 
 * @param[in] trajectory_id 轨迹id
 * @param[in] sensor_id imu的topic名字
 * @param[in] msg imu的ros格式的数据
 */
void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  // extrapolators_使用里程计数据进行位姿预测
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  // 根据配置,是否将传感器数据跳过
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}
// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}
// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename,
                          const bool include_unfinished_submaps) {
  absl::MutexLock lock(&mutex_);
  CHECK(
      map_builder_bridge_->SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";
}

// 加载pbstream文件
void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->LoadState(state_filename, load_frozen_state);
}

// TODO: find ROS equivalent to ros::master::getTopics
// 检查设置的topic名字是否在ros中存在, 不存在则报错
void Node::MaybeWarnAboutTopicMismatch() {
   // note: 使用ros的master的api进行topic名字的获取
//  ::ros::master::V_TopicInfo ros_topics;
//  ::ros::master::getTopics(ros_topics);

//  std::set<std::string> published_topics;
//  std::stringstream published_topics_string;
  // 获取ros中的实际topic的全局名称,resolveName()是获取全局名称
//  for (const auto& it : ros_topics) {
        // 获取实际订阅的topic名字
//    std::string resolved_topic = node_handle_.resolveName(it.name, false);
      // 如果设置的topic名字,在ros中不存在,则报错
//    published_topics.insert(resolved_topic);
//    published_topics_string << resolved_topic << ",";
//  }
//  bool print_topics = false;
//  for (const auto& entry : subscribers_) {
//    int trajectory_id = entry.first;
//    for (const auto& subscriber : entry.second) {
//      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
//      if (published_topics.count(resolved_topic) == 0) {
//        LOG(WARNING) << "Expected topic \"" << subscriber.topic
//                     << "\" (trajectory " << trajectory_id << ")"
//                     << " (resolved topic \"" << resolved_topic << "\")"
//                     << " but no publisher is currently active.";
//        print_topics = true;
//      }
//    }
//  }
  // 告诉使用者哪些topic可用
//  if (print_topics) {
//    LOG(WARNING) << "Currently available topics are: "
//                 << published_topics_string.str();
//  }
}

}  // namespace cartographer_ros
