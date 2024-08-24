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

#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/internal/ceres_solver_options.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/optimization/ceres_pose.h"
#include "cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_2d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_2d.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using ::cartographer::mapping::optimization::CeresPose;
using LandmarkNode = ::cartographer::mapping::PoseGraphInterface::LandmarkNode;
using TrajectoryData =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryData;

// ���ݽڵ��ʱ���gps���ݽ��в�ֵ, ��ȡ���ʱ�̵�gps���ݵ�λ�� For fixed frame pose.
std::unique_ptr<transform::Rigid3d> Interpolate(
    const sensor::MapByTime<sensor::FixedFramePoseData>& map_by_time,
    const int trajectory_id, const common::Time time) {
  const auto it = map_by_time.lower_bound(trajectory_id, time);
  if (it == map_by_time.EndOfTrajectory(trajectory_id) ||
      !it->pose.has_value()) {
    return nullptr;
  }
  if (it == map_by_time.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid3d>(it->pose.value());
    }
    return nullptr;
  }
  // �����ʱ��һǰһ���gps���ݽ��в�ֵ
  const auto prev_it = std::prev(it);
  if (prev_it->pose.has_value()) {
    return absl::make_unique<transform::Rigid3d>(
        Interpolate(transform::TimestampedTransform{prev_it->time,
                                                    prev_it->pose.value()},
                    transform::TimestampedTransform{it->time, it->pose.value()},
                    time)
            .transform);
  }
  return nullptr;
}

// Converts a pose into the 3 optimization variable format used for Ceres:
// translation in x and y, followed by the rotation angle representing the
// orientation.
// Rigid2dתarray����
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           pose.normalized_angle()}};
}

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
// array����תtransform::Rigid2d 
transform::Rigid2d ToPose(const std::array<double, 3>& values) {
  return transform::Rigid2d({values[0], values[1]}, values[2]);
}

// Selects a trajectory node closest in time to the landmark observation and
// applies a relative transform from it.
// ����landmark���ݵ�ʱ���2���ڵ�λ�˽��в�ֵ, �õ����ʱ�̵�global����ϵ�µ�λ��
transform::Rigid3d GetInitialLandmarkPose(
    const LandmarkNode::LandmarkObservation& observation,
    const NodeSpec2D& prev_node, const NodeSpec2D& next_node,
    const std::array<double, 3>& prev_node_pose,
    const std::array<double, 3>& next_node_pose) {
  const double interpolation_parameter =
      common::ToSeconds(observation.time - prev_node.time) /
      common::ToSeconds(next_node.time - prev_node.time);
  // ����landmark���ݵ�ʱ���2���ڵ�λ�˽��в�ֵ, �õ����ʱ�̵�tracking_frame��global����ϵ�µ�λ��
  const std::tuple<std::array<double, 4>, std::array<double, 3>>
      rotation_and_translation =
          InterpolateNodes2D(prev_node_pose.data(), prev_node.gravity_alignment,
                             next_node_pose.data(), next_node.gravity_alignment,
                             interpolation_parameter);
  // ��landmark�����ݴ�tracking_frame�µ�λ��ת��global����ϵ��
  return transform::Rigid3d::FromArrays(std::get<0>(rotation_and_translation),
                                        std::get<1>(rotation_and_translation)) *
         observation.landmark_to_tracking_transform;
}

// landmark���� �� ͨ��2���ڵ�λ�˲�ֵ���������λ�� �Ĳ�ֵ��Ϊ�в���
void AddLandmarkCostFunctions(
    const std::map<std::string, LandmarkNode>& landmark_nodes,
    const MapById<NodeId, NodeSpec2D>& node_data,
    MapById<NodeId, std::array<double, 3>>* C_nodes,
    std::map<std::string, CeresPose>* C_landmarks, ceres::Problem* problem,
    double huber_scale) {
  // �����е�landmark���б���, ���landmark�Ĳв�
  for (const auto& landmark_node : landmark_nodes) {
    for (const auto& observation : landmark_node.second.landmark_observations) {
      const std::string& landmark_id = landmark_node.first;
      const auto& begin_of_trajectory =
          node_data.BeginOfTrajectory(observation.trajectory_id);
      // The landmark observation was made before the trajectory was created.
      if (observation.time < begin_of_trajectory->data.time) {
        continue;
      }
      // Find the trajectory nodes before and after the landmark observation.
      // �ҵ�landmark�۲�ʱ���ĵ�һ���ڵ�
      auto next =
          node_data.lower_bound(observation.trajectory_id, observation.time);
      // The landmark observation was made, but the next trajectory node has
      // not been added yet.
      if (next == node_data.EndOfTrajectory(observation.trajectory_id)) {
        continue;
      }
      if (next == begin_of_trajectory) {
        next = std::next(next);
      }
      // �ҵ�landmark�۲�ʱ��ǰһ���ڵ�
      auto prev = std::prev(next);
      // Add parameter blocks for the landmark ID if they were not added before.
      std::array<double, 3>* prev_node_pose = &C_nodes->at(prev->id);
      std::array<double, 3>* next_node_pose = &C_nodes->at(next->id);
      // ���landmark_id��Ӧ������û���뵽C_landmarks��, ������ӽ�ȥ
      if (!C_landmarks->count(landmark_id)) {
        // ������Ż����λ�˾����Ż����λ��, û�о͸���ʱ���ֵ�����һ��λ��
        // starting_point������֡landmark���ݶ�Ӧ��tracking_frame��global����ϵ�µ�λ��
        const transform::Rigid3d starting_point =
            landmark_node.second.global_landmark_pose.has_value()
                ? landmark_node.second.global_landmark_pose.value()
                : GetInitialLandmarkPose(observation, prev->data, next->data,
                                         *prev_node_pose, *next_node_pose);
#if CERES_VERSION_MAJOR > 2 || CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1
        // ��landmark���ݷ���C_landmarks
        C_landmarks->emplace(
            landmark_id,
            // ��landmark���ݶ�Ӧ�Ľڵ��ƽ������ת��Ϊ�Ż��������뵽problem��
            CeresPose(starting_point, nullptr /* translation_manifold */,
                      absl::make_unique<ceres::QuaternionManifold>(),
                      problem));
#else
        C_landmarks->emplace(
            landmark_id,
            CeresPose(starting_point, nullptr /* translation_parametrization */,
                      absl::make_unique<ceres::QuaternionParameterization>(),
                      problem));
#endif
        // Set landmark constant if it is frozen.
        if (landmark_node.second.frozen) {
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).translation());
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).rotation());
        }
      }
      // Step: �ڶ��ֲв� landmark������ڵ�λ�˼���������任 �� landmark�۲� �Ĳ�ֵ��Ϊ�в���
      problem->AddResidualBlock(
          LandmarkCostFunction2D::CreateAutoDiffCostFunction(
              observation, prev->data, next->data),
          new ceres::HuberLoss(huber_scale), 
          prev_node_pose->data(),
          next_node_pose->data(), 
          C_landmarks->at(landmark_id).rotation(),
          C_landmarks->at(landmark_id).translation());
    }
  }
}

}  // namespace

OptimizationProblem2D::OptimizationProblem2D(
    const proto::OptimizationProblemOptions& options)
    : options_(options) {}

OptimizationProblem2D::~OptimizationProblem2D() {}

// 2D�Ż��в�ʹ��IMU����, �������Ǻ����ⲿ�ֽӿ�
void OptimizationProblem2D::AddImuData(const int trajectory_id,
                                       const sensor::ImuData& imu_data) {
  // IMU data is not used in 2D optimization, so we ignore this part of the
  // interface.
}

// �����̼�����
void OptimizationProblem2D::AddOdometryData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  odometry_data_.Append(trajectory_id, odometry_data);
}

// ���gps����
void OptimizationProblem2D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  fixed_frame_pose_data_.Append(trajectory_id, fixed_frame_pose_data);
}

// ���Ż������м���ڵ�λ������
void OptimizationProblem2D::AddTrajectoryNode(const int trajectory_id,
                                              const NodeSpec2D& node_data) {
  node_data_.Append(trajectory_id, node_data);
  trajectory_data_[trajectory_id];
}

// ����TrajectoryData ֻ��PoseGraph2D::SetTrajectoryDataFromProto������һ��
void OptimizationProblem2D::SetTrajectoryData(
    int trajectory_id, const TrajectoryData& trajectory_data) {
  trajectory_data_[trajectory_id] = trajectory_data;
}

// ���ýڵ�λ��
void OptimizationProblem2D::InsertTrajectoryNode(const NodeId& node_id,
                                                 const NodeSpec2D& node_data) {
  node_data_.Insert(node_id, node_data);
  trajectory_data_[node_id.trajectory_id];
}

// ɾ���ڵ�, �ڴ���λʱʹ��
void OptimizationProblem2D::TrimTrajectoryNode(const NodeId& node_id) {
  // ��������ڵ��ʱ���ɾ������������
  empty_imu_data_.Trim(node_data_, node_id);
  odometry_data_.Trim(node_data_, node_id);
  fixed_frame_pose_data_.Trim(node_data_, node_id);
  // ɾ���ڵ�
  node_data_.Trim(node_id);
  // ������ݿ��˾Ͱɹ켣ɾ����
  if (node_data_.SizeOfTrajectoryOrZero(node_id.trajectory_id) == 0) {
    trajectory_data_.erase(node_id.trajectory_id);
  }
}

// �����ͼλ��
void OptimizationProblem2D::AddSubmap(
    const int trajectory_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Append(trajectory_id, SubmapSpec2D{global_submap_pose});
}

// �����ͼλ��
void OptimizationProblem2D::InsertSubmap(
    const SubmapId& submap_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Insert(submap_id, SubmapSpec2D{global_submap_pose});
}

// ɾ��ָ��id����ͼλ��, �ڴ���λʱʹ��
void OptimizationProblem2D::TrimSubmap(const SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}

// ��������������
void OptimizationProblem2D::SetMaxNumIterations(
    const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

/**
 * @brief ��Ż����Ⲣ�������
 * 
 * @param[in] constraints ���е�Լ������
 * @param[in] trajectories_state �켣��״̬
 * @param[in] landmark_nodes landmark����
 */
void OptimizationProblem2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, PoseGraphInterface::TrajectoryState>&
        trajectories_state,
    const std::map<std::string, LandmarkNode>& landmark_nodes) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  // ��¼������FROZEN״̬�Ĺ켣id
  std::set<int> frozen_trajectories;
  for (const auto& it : trajectories_state) {
    if (it.second == PoseGraphInterface::TrajectoryState::FROZEN) {
      frozen_trajectories.insert(it.first);
    }
  }

  // �����Ż��������
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  // TODO(hrapp): Move ceres data into SubmapSpec.
  // ceres��Ҫdouble��ָ��, std::array��ת��ԭʼָ�����ʽ
  MapById<SubmapId, std::array<double, 3>> C_submaps;
  MapById<NodeId, std::array<double, 3>> C_nodes;
  std::map<std::string, CeresPose> C_landmarks;
  bool first_submap = true;

  // ����Ҫ�Ż�����ͼλ������Ϊ�Ż�����
  for (const auto& submap_id_data : submap_data_) {
    // submap_id�Ĺ켣 �Ƿ��� �Ѿ�����Ĺ켣
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    // ����ͼ��global_pose����C_submaps��
    C_submaps.Insert(submap_id_data.id,
                     FromPose(submap_id_data.data.global_pose));
    // c++11: std::array::data() ����ָ����������е�һ��Ԫ�ص�ָ��
    // Step: �����Ҫ�Ż������� ������ʽ��Ӳ�����,����ж���Ĳ�������ȷ�Լ��
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);

    if (first_submap || frozen) {
      first_submap = false;
      // Fix the pose of the first submap or all submaps of a frozen
      // trajectory.
      // Step: ����ǵ�һ����ͼ, �������Ѿ�����Ĺ켣�е���ͼ, ���Ż������ͼλ��
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
  }
  
  // ����Ҫ�Ż��Ľڵ�λ������Ϊ�Ż�����
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    // ���ڵ��global_pose_2d����C_nodes��
    C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.global_pose_2d));
    problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
    // ��һ���ڵ��λ��Ҳ��Ҫ�Ż��ı���, ���ǹ̶���
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
    }
  }
  
  // Step: ��һ�ֲв� ���ڵ�����ͼԭ����global����ϵ�µ����λ�� �� Լ�� �Ĳ�ֵ��Ϊ�в���
  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        // ����SPA�����еĹ�ʽ������Ĳв��CostFunction
        CreateAutoDiffSpaCostFunction(constraint.pose),
        // Loop closure constraints should have a loss function.
        // Ϊ�ջ�Լ���ṩһ��Huber�ĺ˺���,���ڽ��ʹ���ıջ��������յ��Ż���������ĸ���Ӱ��
        constraint.tag == Constraint::INTER_SUBMAP // �˺���
            ? new ceres::HuberLoss(options_.huber_scale()) // param: huber_scale
            : nullptr,
        C_submaps.at(constraint.submap_id).data(), // 2���Ż�����
        C_nodes.at(constraint.node_id).data());
  }
  
  // Add cost functions for landmarks.
  // Step: landmark���� �� ͨ��2���ڵ�λ�˲�ֵ���������λ�� �Ĳ�ֵ��Ϊ�в���
  AddLandmarkCostFunctions(landmark_nodes, node_data_, &C_nodes, &C_landmarks,
                           &problem, options_.huber_scale());
  
  // Add penalties for violating odometry or changes between consecutive nodes
  // if odometry is not available.
  // �����̼Ʋ�����, �����Υ����̼ƵĴ����������ڵ�֮��ĸ���

  // ��������켣, �����̼���local����Ĳв�
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    // ��ȡÿ���ڵ�Ĺ켣id
    const int trajectory_id = node_it->id.trajectory_id;
    // ��ȡ�����켣�����һ��λ�õĵ�����
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    // ����켣��frozen��, �����账��ֱ������
    if (frozen_trajectories.count(trajectory_id) != 0) {
      node_it = trajectory_end;
      continue;
    }

    auto prev_node_it = node_it;
    // ����һ���켣�����нڵ�, �����̼���local����Ĳв�
    for (++node_it; node_it != trajectory_end; ++node_it) {
      const NodeId first_node_id = prev_node_it->id;
      const NodeSpec2D& first_node_data = prev_node_it->data;
      prev_node_it = node_it;

      const NodeId second_node_id = node_it->id;
      const NodeSpec2D& second_node_data = node_it->data;

      // ����ڵ������������, ����
      if (second_node_id.node_index != first_node_id.node_index + 1) {
        continue;
      }

      // Add a relative pose constraint based on the odometry (if available).
      // ������̼����ݽ��в�ֵ�õ���2���ڵ�������任
      std::unique_ptr<transform::Rigid3d> relative_odometry =
          CalculateOdometryBetweenNodes(trajectory_id, first_node_data,
                                        second_node_data);
      // Step: �����ֲв� �ڵ���ڵ����global����ϵ�µ��������任 �� ͨ����̼����ݲ�ֵ�����������任 �Ĳ�ֵ��Ϊ�в���
      // ���������̼��������һ���в�
      if (relative_odometry != nullptr) {
        problem.AddResidualBlock(
            CreateAutoDiffSpaCostFunction(Constraint::Pose{
                *relative_odometry, options_.odometry_translation_weight(),
                options_.odometry_rotation_weight()}),
            nullptr /* loss function */, 
            C_nodes.at(first_node_id).data(),
            C_nodes.at(second_node_id).data());
      }

      // Add a relative pose constraint based on consecutive local SLAM poses.
      // ��������2���ڵ���local����ϵ�µ�����任
      const transform::Rigid3d relative_local_slam_pose =
          transform::Embed3D(first_node_data.local_pose_2d.inverse() *
                             second_node_data.local_pose_2d);
      // Step: �����ֲв� �ڵ���ڵ����global����ϵ�µ��������任 �� ����2���ڵ���local����ϵ�µ��������任 �Ĳ�ֵ��Ϊ�в���
      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(
              Constraint::Pose{relative_local_slam_pose,
                               options_.local_slam_pose_translation_weight(),
                               options_.local_slam_pose_rotation_weight()}),
          nullptr /* loss function */, 
          C_nodes.at(first_node_id).data(),
          C_nodes.at(second_node_id).data());
    }
  }

  // ��������켣, ���gps�Ĳв�
  std::map<int, std::array<double, 3>> C_fixed_frames;
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (!fixed_frame_pose_data_.HasTrajectory(trajectory_id)) {
      node_it = trajectory_end;
      continue;
    }

    const TrajectoryData& trajectory_data = trajectory_data_.at(trajectory_id);
    bool fixed_frame_pose_initialized = false;
    
    // ����һ���켣�����нڵ�, ���gps�Ĳв�
    for (; node_it != trajectory_end; ++node_it) {
      const NodeId node_id = node_it->id;
      const NodeSpec2D& node_data = node_it->data;

      // ���ݽڵ��ʱ���gps���ݽ��в�ֵ, ��ȡ���ʱ�̵�gps���ݵ�λ��
      const std::unique_ptr<transform::Rigid3d> fixed_frame_pose =
          Interpolate(fixed_frame_pose_data_, trajectory_id, node_data.time);
      // Ҫ�ҵ���һ����Ч�����ݲ��ܽ��вв�ļ���
      if (fixed_frame_pose == nullptr) {
        continue;
      }
      // gps���ݵ�gps��һ�����ݼ������任
      const Constraint::Pose constraint_pose{
          *fixed_frame_pose, options_.fixed_frame_pose_translation_weight(),
          options_.fixed_frame_pose_rotation_weight()};

      // ����gps����ϵԭ����global����ϵ�µ�����
      if (!fixed_frame_pose_initialized) {
        transform::Rigid2d fixed_frame_pose_in_map;
        // ���������gps���ݵ�ԭ��
        if (trajectory_data.fixed_frame_origin_in_map.has_value()) {
          fixed_frame_pose_in_map = transform::Project2D(
              trajectory_data.fixed_frame_origin_in_map.value());
        } 
        // ��һ���Ż�֮ǰִ�е�������
        else {
          // ����gps��һ��������global����ϵ�µ�����, �൱��gps���ݵ�����ϵԭ����global����ϵ�µ�����
          fixed_frame_pose_in_map =
              node_data.global_pose_2d *
              transform::Project2D(constraint_pose.zbar_ij).inverse();
        }

        // ����gps����ϵԭ����global����ϵ�µ�����
        C_fixed_frames.emplace(trajectory_id,
                               FromPose(fixed_frame_pose_in_map));
        fixed_frame_pose_initialized = true;
      }

      // Step: �����ֲв� �ڵ���gps����ϵԭ����global����ϵ�µ��������任 �� ͨ��gps���ݽ��в�ֵ�õ����������任 �Ĳ�ֵ��Ϊ�в���
      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(constraint_pose),
          options_.fixed_frame_pose_use_tolerant_loss()
              ? new ceres::TolerantLoss(
                    options_.fixed_frame_pose_tolerant_loss_param_a(),
                    options_.fixed_frame_pose_tolerant_loss_param_b())
              : nullptr,
          C_fixed_frames.at(trajectory_id).data(), // ���Զ�����AddParameterBlock
          C_nodes.at(node_id).data());
    }
  }

  // Solve. �������
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);

  // ����������Ż���log���, �����ceres�ı���
  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
  }

  // ���Ż�����������ݽ��и��� Store the result.
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        ToPose(C_submap_id_data.data);
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose_2d =
        ToPose(C_node_id_data.data);
  }
  for (const auto& C_fixed_frame : C_fixed_frames) {
    trajectory_data_.at(C_fixed_frame.first).fixed_frame_origin_in_map =
        transform::Embed3D(ToPose(C_fixed_frame.second));
  }
  for (const auto& C_landmark : C_landmarks) {
    landmark_data_[C_landmark.first] = C_landmark.second.ToRigid();
  }
}

// ����ʱ�����̼����ݽ��в�ֵ, ��ȡ���ʱ�̵���̼����ݵ�λ��
std::unique_ptr<transform::Rigid3d> OptimizationProblem2D::InterpolateOdometry(
    const int trajectory_id, const common::Time time) const {
  // �ҵ�ʱ���ϵ�һ�����ڵ���time����̼����ݵĵ�����
  const auto it = odometry_data_.lower_bound(trajectory_id, time);
  if (it == odometry_data_.EndOfTrajectory(trajectory_id)) {
    return nullptr;
  }
  if (it == odometry_data_.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid3d>(it->pose);
    }
    return nullptr;
  }
  // ǰһ����̼�����
  const auto prev_it = std::prev(it);
  // ����ʱ��������Բ�ֵ
  return absl::make_unique<transform::Rigid3d>(
      Interpolate(transform::TimestampedTransform{prev_it->time, prev_it->pose},
                  transform::TimestampedTransform{it->time, it->pose}, time)
          .transform);
}

/**
 * @brief ������̼�������������ڵ����������任
 * 
 * @param[in] trajectory_id �켣��id
 * @param[in] first_node_data ǰһ���ڵ�����
 * @param[in] second_node_data ��һ���ڵ�����
 * @return std::unique_ptr<transform::Rigid3d> �����ڵ������任
 */
std::unique_ptr<transform::Rigid3d>
OptimizationProblem2D::CalculateOdometryBetweenNodes(
    const int trajectory_id, const NodeSpec2D& first_node_data,
    const NodeSpec2D& second_node_data) const {

  if (odometry_data_.HasTrajectory(trajectory_id)) {
    // ��ֵ�õ�timeʱ�̵���̼�����
    const std::unique_ptr<transform::Rigid3d> first_node_odometry =
        InterpolateOdometry(trajectory_id, first_node_data.time);
    const std::unique_ptr<transform::Rigid3d> second_node_odometry =
        InterpolateOdometry(trajectory_id, second_node_data.time);

    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      // ������̼�����������������任
      // ��Ҫע�����, ʵ������optimization_problem��, node��λ�˶���2dƽ���ϵ�
      // ��odometry��pose�Ǵ���̬��, ���Ҫ�����ټƲ�ֵ������λ��ת��ƽ����
      transform::Rigid3d relative_odometry =
          transform::Rigid3d::Rotation(first_node_data.gravity_alignment) *
          first_node_odometry->inverse() * (*second_node_odometry) *
          transform::Rigid3d::Rotation(
              second_node_data.gravity_alignment.inverse());

      return absl::make_unique<transform::Rigid3d>(relative_odometry);
    }
  }

  return nullptr;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
