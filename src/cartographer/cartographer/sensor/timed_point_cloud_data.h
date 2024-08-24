/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace sensor {

// ʱ��ͬ��ǰ�ĵ���
struct TimedPointCloudData {
  common::Time time;        // �������һ�����ʱ��
  Eigen::Vector3f origin;   // ��tracking_frame_���״�����ϵ������任Ϊԭ��
  TimedPointCloud ranges;   // ���ݵ�ļ���, ÿ�����ݵ����xyz��time, time�Ǹ���
  // 'intensities' has to be same size as 'ranges', or empty.
  std::vector<float> intensities; // �յ�
};

// ʱ��ͬ����ĵ���
struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    TimedRangefinderPoint point_time;   // ��ʱ����ĵ������ݵ������ xyz
    float intensity;                    // ǿ��ֵ
    size_t origin_index;                // ���ڵڼ���origins�ĵ�
  };
  common::Time time;                    // ���Ƶ�ʱ��
  std::vector<Eigen::Vector3f> origins; // �������ɼ����������, ÿ�����Ƶ�ԭ��
  std::vector<RangeMeasurement> ranges; // ���ݵ�ļ���
};

// Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data);

// Converts 'proto' to TimedPointCloudData.
TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
