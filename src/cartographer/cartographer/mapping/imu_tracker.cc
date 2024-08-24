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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/**
 * @brief Construct a new Imu Tracker:: Imu Tracker object
 * 
 * @param[in] imu_gravity_time_constant ���ֵ��2d��3d����¶�Ϊ10
 * @param[in] time 
 */
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()), // ��ʼ�����
      gravity_vector_(Eigen::Vector3d::UnitZ()),    // ���������ʼ��Ϊ[0,0,1]
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

/**
 * @brief Ԥ���timeʱ�̵���̬����������
 * 
 * @param[in] time ҪԤ���ʱ��
 */
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  // ��һʱ�̵Ľ��ٶȳ���ʱ��,�õ���ǰʱ���������һʱ�̵�Ԥ�����̬�仯��,��ת������Ԫ��
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  // ʹ����һʱ�̵���̬ orientation_ ������̬�仯��, �õ���ǰʱ�̵�Ԥ�������̬
  orientation_ = (orientation_ * rotation).normalized();

  // ����Ԥ�������̬�仯��,Ԥ����ת������Լ��ٶȵ�ֵ
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  // ����ʱ��
  time_ = time;
}

/**
 * @brief �������Լ��ٶȵ�ֵ,�����������ķ������һʱ�̵���̬����У׼
 * 
 * @param[in] imu_linear_acceleration imu���߼��ٶȵĴ�С
 */
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  // ָ������ƽ���� exponential moving average
 
  // Step: 1 ��delta_t, delta_t��ʼʱ��Ϊinfinity, ֮��Ϊtime_-last_linear_acceleration_time_
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;

  // Step: 2 ��alpha, alpha=1-e^(-delta_t/10)
  // delta_tԽ��, alphaԽ��
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);

  // Step: 3 ��֮ǰ���߼��ٶ��뵱ǰ������߼��ٶȽ����ں�, �������ָ������ƽ����

  // ָ����ȷ��Ȩ��, ��Ϊ�������Ĵ���, ʱ���Խ��, ��ǰ�����Լ��ٶȵ�Ȩ��Խ��
  // �����gravity_vector_�ĳ����Լ��ٶȸ�����һЩ
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
      
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  // Step: 4 ��� ���Լ��ٶȵ�ֵ �� ����һʱ����̬��������Լ��ٶ� �����ת��
  const Eigen::Quaterniond rotation = FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());

  // Step: 5 ʹ�������ת����У׼��ǰ����̬
  orientation_ = (orientation_ * rotation).normalized();

  // note: glog CHECK_GT: ��һ������Ҫ���ڵڶ�������
  // ������Լ��ٶ�����̬��������ȫ��ȷ,������ߵĳ˻�Ӧ���� 0 0 1
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

// ���½��ٶ�
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
