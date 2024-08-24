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

#include "cartographer/mapping/internal/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

/**
 * @brief Construct a new Collated Trajectory Builder:: Collated Trajectory Builder object
 * 
 * @param[in] trajectory_options �켣�Ĳ�������
 * @param[in] sensor_collator �����������������,��2������
 * @param[in] trajectory_id �����ɵĹ켣��id
 * @param[in] expected_sensor_ids ������Ҫ��topic�����ֵļ���
 * @param[in] wrapped_trajectory_builder ������slam GlobalTrajectoryBuilder
 */
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    const proto::TrajectoryBuilderOptions& trajectory_options,
    sensor::CollatorInterface* const sensor_collator, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      
      // �������������� configuration_files/trajectory_builder.lua ��
      // collate_landmarks Ϊ false, ��Ҫ��landmark���ݷ��뵽����������
      collate_landmarks_(trajectory_options.collate_landmarks()),
      // collate_fixed_frame Ϊ true, ��gps���ݷ�������������
      collate_fixed_frame_(trajectory_options.collate_fixed_frame()),
      
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now()) {
        
  // ��ȡtopic������, �����ݲ������þ����Ƿ����LANDMARK��gps��topic
  absl::flat_hash_set<std::string> expected_sensor_id_strings;
  for (const auto& sensor_id : expected_sensor_ids) {
    // collate_landmarks Ϊ false, sensor_collator_������LANDMARK����
    if (sensor_id.type == SensorId::SensorType::LANDMARK &&
        !collate_landmarks_) {
      continue;
    }
    // collate_fixed_frame Ϊ true, sensor_collator_����gps����
    if (sensor_id.type == SensorId::SensorType::FIXED_FRAME_POSE &&
        !collate_fixed_frame_) {
      continue;
    }
    expected_sensor_id_strings.insert(sensor_id.id);
  }

  // sensor::Collator�ĳ�ʼ��
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
}

// �����ݴ���sensor_collator_��AddSensorData��������
void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}

/**
 * @brief ���� ����ʱ��˳��ַ������Ĵ���������
 * 
 * @param[in] sensor_id ��������topic������
 * @param[in] data ��Ҫ���������(Data�Ǹ���ģ��,�ɴ�����ֲ�ͬ�������͵�����)
 */
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);
  // �Ҳ������½�һ��
  if (it == rate_timers_.end()) {
    // map::emplace()����һ��pair
    // emplace().first��ʾ�²���Ԫ�ػ���ԭʼλ�õĵ�����
    // emplace().second��ʾ����ɹ�,ֻ����key��map�в�����ʱ�Ų���ɹ�
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, 
                 std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
  
  // �����ݶ��н��и���
  it->second.Pulse(data->GetTime());

  // if (std::chrono::steady_clock::now() - last_logging_time_ >
  //     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
  //   for (const auto& pair : rate_timers_) {
  //     LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
  //   }
  //   last_logging_time_ = std::chrono::steady_clock::now();
  // }

  // Ҳ������cartoʱ�����Ϣ��
  // [ INFO]: collated_trajectory_builder.cc:72] imu rate: 10.00 Hz 1.00e-01 s +/- 4.35e-05 s (pulsed at 100.44% real time)
  // [ INFO]: collated_trajectory_builder.cc:72] scan rate: 19.83 Hz 5.04e-02 s +/- 4.27e-05 s (pulsed at 99.82% real time)

  // ������õ��������� GlobalTrajectoryBuilder�е�AddSensorData()�����н���ʹ��
  data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());
}

}  // namespace mapping
}  // namespace cartographer
