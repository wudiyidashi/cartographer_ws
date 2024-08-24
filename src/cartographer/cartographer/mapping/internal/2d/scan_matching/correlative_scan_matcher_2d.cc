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

#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"

#include <cmath>

#include "cartographer/common/math.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// ���캯��
SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const sensor::PointCloud& point_cloud,
                                   const double resolution)
    : resolution(resolution) {
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  float max_scan_range = 3.f * resolution;

  // ��� point_cloud ���״����ݵ� ����ֵ����Զ��ľ��룩
  for (const sensor::RangefinderPoint& point : point_cloud) {
    const float range = point.position.head<2>().norm();
    max_scan_range = std::max(range, max_scan_range);
  }

  // ����������Ĺ�ʽ ��ýǶȷֱ��� angular_perturbation_step_size
  const double kSafetyMargin = 1. - 1e-3;
  angular_perturbation_step_size =
      kSafetyMargin * std::acos(1. - common::Pow2(resolution) /
                                         (2. * common::Pow2(max_scan_range)));

  // ��Χ���Էֱ��ʵõ�����
  num_angular_perturbations =
      std::ceil(angular_search_window / angular_perturbation_step_size);
  // num_scans��Ҫ������ת���Ƶĸ���, �� num_angular_perturbations ������2��
  num_scans = 2 * num_angular_perturbations + 1;

  // XY�����������Χ, ��λ�Ƕ��ٸ�դ��
  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution);

  // linear_bounds ��������ȷ��ÿһ�����Ƶ������С�߽�
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

// For testing.
SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(angular_perturbation_step_size),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1) {
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

// ����ÿһ֡���� �ڱ�֤���һ�������ڵ�ͼ��Χ��ʱ ������ƶ���Χ
void SearchParameters::ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                                   const CellLimits& cell_limits) {
  CHECK_EQ(scans.size(), num_scans);
  CHECK_EQ(linear_bounds.size(), num_scans);

  // �������ɵ���ת��ĺܶ�scan
  for (int i = 0; i != num_scans; ++i) {
    Eigen::Array2i min_bound = Eigen::Array2i::Zero();
    Eigen::Array2i max_bound = Eigen::Array2i::Zero();

    // �Ե��Ƶ�ÿһ������б���, ȷ����֡���Ƶ������С����������
    for (const Eigen::Array2i& xy_index : scans[i]) {
      // Array2i.min�������� ��ȡ��ӦԪ�ص���Сֵ����µ�Array2i
      min_bound = min_bound.min(-xy_index);
      max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                               cell_limits.num_y_cells - 1) -
                                xy_index);
    }

    // ����ÿһ֡���� �ڱ�֤���һ�������ڵ�ͼ��Χ��ʱ ������ƶ���Χ
    linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
    linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
    linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
    linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
  }
}

/*
void test_ShrinkToFit()
{
  std::vector<Eigen::Array2i> scan;
  scan.push_back(Eigen::Array2i(1, 1));
  scan.push_back(Eigen::Array2i(10, 1));
  scan.push_back(Eigen::Array2i(1, 10));
  scan.push_back(Eigen::Array2i(10, 10));

  Eigen::Array2i min_bound = Eigen::Array2i::Zero();
  Eigen::Array2i max_bound = Eigen::Array2i::Zero();
  int min_x = -140, max_x = 140, min_y = -140, max_y = 140;
  int num_x_cells = 100, num_y_cells = 100;

  // �Ե��Ƶ�ÿһ������б���, ȷ����֡���Ƶ������С����������
  for (const Eigen::Array2i &xy_index : scan)
  {
    // Array2i.min�������� ��ȡ��ӦԪ�ص���Сֵ����µ�Array2i
    min_bound = min_bound.min(-xy_index);
    max_bound = max_bound.max(Eigen::Array2i(num_x_cells - 1,
                                             num_y_cells - 1) -
                              xy_index);
  }

  // ÿһ֡scan�������С����������
  min_x = std::max(min_x, min_bound.x());
  max_x = std::min(max_x, max_bound.x());
  min_y = std::max(min_y, min_bound.y());
  max_y = std::min(max_y, max_bound.y());
}
*/

// ���ɰ��ղ�ͬ�Ƕ���ת��ĵ��Ƽ���
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters) {
  std::vector<sensor::PointCloud> rotated_scans;
  // ���� num_scans ����ת��ĵ���
  rotated_scans.reserve(search_parameters.num_scans);
  // ��ʼ�Ƕ�
  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;
  // ���б�����������ת��ͬ�ǶȺ�ĵ��Ƽ���
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size) {
    // �� point_cloud ��Z����ת��delta_theta
    rotated_scans.push_back(sensor::TransformPointCloud(
        point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                         delta_theta, Eigen::Vector3f::UnitZ()))));
  }
  return rotated_scans;
}

// ����ת��ĵ��Ƽ��ϰ���Ԥ�����ƽ��������ƽ��, ��ȡƽ�ƺ�ĵ��ڵ�ͼ�е�����
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation) {
  // discrete_scans��size Ϊ ��ת�ĵ��Ƶĸ���
  std::vector<DiscreteScan2D> discrete_scans;
  discrete_scans.reserve(scans.size());

  for (const sensor::PointCloud& scan : scans) {
    // discrete_scans�е�ÿһ�� DiscreteScan2D ��size����Ϊ��һ֡���������е�ĸ���
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());

    // �����е�ÿһ�������ƽ��, ��ȡƽ�ƺ��դ������
    for (const sensor::RangefinderPoint& point : scan) {
      // ��scan�е�ÿ�������ƽ��
      const Eigen::Vector2f translated_point =
          Eigen::Affine2f(initial_translation) * point.position.head<2>();

      // ����ת��ĵ� ��Ӧ��դ�����������discrete_scans
      discrete_scans.back().push_back(
          map_limits.GetCellIndex(translated_point));
    }
  }
  return discrete_scans;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
