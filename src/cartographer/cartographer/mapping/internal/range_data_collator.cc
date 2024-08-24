/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

constexpr float RangeDataCollator::kDefaultIntensityValue;

/**
 * @brief ����״����ݵ�ʱ��ͬ��
 * 
 * @param[in] sensor_id �״����ݵĻ���
 * @param[in] timed_point_cloud_data �״�����
 * @return sensor::TimedPointCloudOriginData ����ʱ�䴦��֮�������
 */
sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    sensor::TimedPointCloudData timed_point_cloud_data) { // ��һ�ο���
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);

  // ��sensor_bridge�����������ݵ�intensitiesΪ��
  timed_point_cloud_data.intensities.resize(
      timed_point_cloud_data.ranges.size(), kDefaultIntensityValue);

  // TODO(gaschler): These two cases can probably be one.
  // ���ͬ����ĵ���, ����û�����, ���ȴ�ͬ��û����ĵ���, ����ǰ���Ʊ���
  if (id_to_pending_data_.count(sensor_id) != 0) {
    // current_end_Ϊ��һ��ʱ��ͬ���Ľ���ʱ��
    // current_start_Ϊ����ʱ��ͬ���Ŀ�ʼʱ��
    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    // ����ʱ��ͬ���Ľ���ʱ��Ϊ��֡�������ݵĽ���ʱ��
    current_end_ = id_to_pending_data_.at(sensor_id).time;
    auto result = CropAndMerge();
    // ���浱ǰ����
    id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
    return result;
  }

  // �Ƚ���ǰ������ӵ� �ȴ�ʱ��ͬ����map��
  id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));

  // �ȵ�range���ݵĻ��ⶼ����֮���ٽ��д���
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    return {};
  }

  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  // �ҵ����д����������������ʱ���(�������һ�����ʱ��)
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  // current_end_�Ǳ���ʱ��ͬ���Ľ���ʱ��
  // �Ǵ�ʱ��ͬ��map�е� ���е����������ʱ���
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}

// ��ʱ����ڵ����ݽ��н�ȡ��ϲ�, ����ʱ��ͬ����ĵ���
sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  // �������еĴ���������
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) {
    // ��ȡ���ݵ�����
    sensor::TimedPointCloudData& data = it->second;
    const sensor::TimedPointCloud& ranges = it->second.ranges;
    const std::vector<float>& intensities = it->second.intensities;

    // �ҵ������� ���һ��ʱ���С��current_start_�ĵ������
    auto overlap_begin = ranges.begin();
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin).time) <
               current_start_) {
      ++overlap_begin;
    }

    // �ҵ������� ���һ��ʱ���С�ڵ���current_end_�ĵ������
    auto overlap_end = overlap_begin;
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end).time) <=
               current_end_) {
      ++overlap_end;
    }

    // ����������ʱ�����ʼʱ����ĵ�, ÿִ��һ��CropAndMerge()��ӡһ��log
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      // ��ȡ�¸����Ƶ�index, ����ǰvector�ĸ���
      std::size_t origin_index = result.origins.size();
      result.origins.push_back(data.origin);  // ����ԭ������

      // ��ȡ�˴�����ʱ���뼯��ʱ��������, 
      const float time_correction =
          static_cast<float>(common::ToSeconds(data.time - current_end_));

      auto intensities_overlap_it =
          intensities.begin() + (overlap_begin - ranges.begin());
      // reserve() ��Ԥ���ռ�ı�ʱ, �Ὣ֮ǰ�����ݿ������µ��ڴ���
      result.ranges.reserve(result.ranges.size() +
                            std::distance(overlap_begin, overlap_end));
      
      // �������
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it, ++intensities_overlap_it) {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{
            *overlap_it, *intensities_overlap_it, origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        // ���ÿ����ʱ�����������, �����һ�����ʱ��Ϊ0
        point.point_time.time += time_correction;  
        result.ranges.push_back(point);
      } // end for
    } // end if

    // Drop buffered points until overlap_end.
    // �������ÿ���㶼����, ��ɽ�������ݽ���ɾ��
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);
    } 
    // ���һ���㶼û��, ���ȷ���, ����һ������
    else if (overlap_end == ranges.begin()) {
      ++it;
    } 
    // ����һ���ֵĵ�
    else {
      const auto intensities_overlap_end =
          intensities.begin() + (overlap_end - ranges.begin());
      // �����˵ĵ�ɾ��, ����ĸ�ֵ�ǿ���
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end()),
          std::vector<float>(intensities_overlap_end, intensities.end())};
      ++it;
    }
  } // end for

  // �Ը��������ĵ��� ����ÿ�����ʱ���С�����������
  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time.time < b.point_time.time;
            });
  return result;
}

}  // namespace mapping
}  // namespace cartographer
