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

#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

/************** SlidingWindowMaximum **************/

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).
// ���������㷨
class SlidingWindowMaximum {
 public:
  // ���ֵ, �ὫС������ֵ������ֵɾ��, �ٽ����ֵ�ŵ����
  void AddValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  // ɾ��ֵ, �����һ��ֵ����Ҫɾ�������ֵ, �����ֵɾ��
  void RemoveValue(const float value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  // ��ȡ���ֵ, ��Ϊ�ǰ���˳��洢��, ��һ��ֵ������
  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.
  std::deque<float> non_ascending_maxima_;
};

}  // namespace

/************** PrecomputationGrid2D **************/

proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions2D options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  return options;
}

// ���첻ͬ�ֱ��ʵĵ�ͼ
PrecomputationGrid2D::PrecomputationGrid2D(
    const Grid2D& grid, const CellLimits& limits, const int width,
    std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      min_score_(1.f - grid.GetMaxCorrespondenceCost()), // 0.1 min_score_
      max_score_(1.f - grid.GetMinCorrespondenceCost()), // 0.9 max_score_
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) {
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);

  const int stride = wide_limits_.num_x_cells;
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  std::vector<float>& intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  
  // ��ÿһ�д����Һ�����һ�λ���, ��������ĵ�ͼ����intermediate(��ʱ����)��
  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    // ��ȡ grid ��x���������: ���Ȼ�ȡ (0, y)
    current_values.AddValue(
        1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(0, y))));

    // Step: 1 ����������x����ʼ�����ͼ, ����ֻ���� ����ֵ
    // intermediate������x + width - 1 + y * stride�ķ�Χ�� [0, width-2] �ټ��� y * stride
    // grid������ x + width �����귶Χ�� [1, width-1]
    for (int x = -width + 1; x != 0; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      if (x + width < limits.num_x_cells) {
        current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                          Eigen::Array2i(x + width, y))));
      }
    }

    // Step: 2 ���������Ѿ���ȫ�ڵ�ͼ����, ��������һ��һ���Ĳ���
    // x + width - 1 + y * stride �ķ�Χ�� [width-1, limits.num_x_cells-2] �ټ��� y * stride
    // grid������ x + width �����귶Χ�� [width, limits.num_x_cells-width-1]
    for (int x = 0; x < limits.num_x_cells - width; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
      current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                        Eigen::Array2i(x + width, y))));
    }

    // Step: 3 �����������ڻ���, һ�μ���һ��ֵ, ����intermediate�Ŀ�ȱ�grid�� width-1
    // x + width - 1 + y * stride �ķ�Χ�� [limits.num_x_cells-1, limits.num_x_cells+width-1] �ټ��� y * stride
    // grid ������ x�ķ�Χ�� [limits.num_x_cells-width, limits.num_x_cells-1]
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
    }
    // ������, ���������ͼ��һ��֮��Ӧ���ǿյ�, ���� ֻ��, һ��һ��, ֻ��, 3������
    current_values.CheckIsEmpty();
  }

  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.

  // ����intermediate��ֵ, ��ÿһ�д��µ�����������һ�λ���, ������������ĵ�ͼcells_
  for (int x = 0; x != wide_limits_.num_x_cells; ++x) {
    SlidingWindowMaximum current_values;

    current_values.AddValue(intermediate[x]);
    for (int y = -width + 1; y != 0; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
}

/* test
void test_SlidingWindowMaximum()
{
  std::vector<int> grid(10, 1);
  grid[2] = 5;
  grid[6] = 8;
  int width = 2, y = 0, stride = 0;
  std::vector<int> intermediate(grid.size() + width - 1, 0);

  SlidingWindowMaximum current_values;
  current_values.AddValue(grid[0]);

  // ����������x����ʼ�����ͼ, ����ֻ���� ����ֵ
  for (int x = -width + 1; x != 0; ++x)
  {
    intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
    current_values.AddValue(grid[x + width]);
  }

  for (int x = 0; x < grid.size() - width; ++x)
  {
    intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
    current_values.RemoveValue(grid[x]);
    current_values.AddValue(grid[x + width]);
  }

  // �����������ڻ���, һ�μ���һ��ֵ, ����intermediate�Ŀ�ȱ�grid�� width-1
  for (int x = grid.size() - width; x != grid.size(); ++x)
  {
    intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
    current_values.RemoveValue(grid[x]);
  }

  std::cout << "intermediate: " << endl;
  for (auto& x : intermediate)
    std::cout << x << " ";
  std::cout << endl;
}
*/

// ������[0.1, 0.9]ת��[0, 255]֮���ֵ
uint8 PrecomputationGrid2D::ComputeCellValue(const float probability) const {
  const int cell_value = common::RoundToInt(
      (probability - min_score_) * (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

// �����ֱ��ʵ�ͼ
PrecomputationGridStack2D::PrecomputationGridStack2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options) {
  CHECK_GE(options.branch_and_bound_depth(), 1);

  // param: branch_and_bound_depth Ĭ��Ϊ7, ȷ�� ���ķֱ���, Ҳ����64��դ��ϳ�һ������
  const int max_width = 1 << (options.branch_and_bound_depth() - 1); // 64
  precomputation_grids_.reserve(options.branch_and_bound_depth());
  
  // �����ͼֵ
  std::vector<float> reusable_intermediate_grid;
  const CellLimits limits = grid.limits().cell_limits();

  // ���������������դ���ͼ����, x���������ԭ��ͼ��max_width-1������
  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);

  // �ֱ����𽥱��, i=0ʱ����Ĭ�Ϸֱ���0.05, i=6ʱ, width=64,Ҳ����64�����Ӻϳ�һ��ֵ
  for (int i = 0; i != options.branch_and_bound_depth(); ++i) {
    const int width = 1 << i;
    // ���첻ͬ�ֱ��ʵĵ�ͼ PrecomputationGrid2D
    precomputation_grids_.emplace_back(grid, limits, width,
                                       &reusable_intermediate_grid);
  }
}

/************** FastCorrelativeScanMatcher2D **************/

// ���캯��
FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options),
      limits_(grid.limits()),
      // ��ֱ��ʵ�ͼ�Ĺ���
      precomputation_grid_stack_(
          absl::make_unique<PrecomputationGridStack2D>(grid, options)) {}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}

/**
 * @brief ���оֲ��������ڵ�Լ������(�Ծֲ���ͼ���лػ����)
 * 
 * @param[in] initial_pose_estimate ����λ��
 * @param[in] point_cloud ԭ��λ��local����ϵԭ�㴦�ĵ���
 * @param[in] min_score ��С��ֵ, ������������᷵��ʧ��
 * @param[out] score ƥ���ĵ÷�
 * @param[out] pose_estimate ƥ���õ���λ��
 * @return true ƥ��ɹ�, ��֮ƥ��ʧ��
 */
bool FastCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // param: linear_search_window angular_search_window 
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

/**
 * @brief ����ȫ���������ڵ�Լ������(��������ͼ���лػ����)
 * 
 * @param[in] point_cloud ԭ��λ��local����ϵԭ�㴦�ĵ���
 * @param[in] min_score ��С��ֵ, ������������᷵��ʧ��
 * @param[out] score ƥ���ĵ÷�
 * @param[out] pose_estimate ƥ���õ���λ��
 * @return true ƥ��ɹ�, ��֮ƥ��ʧ��
 */
bool FastCorrelativeScanMatcher2D::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // Compute a search window around the center of the submap that includes it
  // fully.
  // �������������ó� xy��Χ��1e6��, �Ƕȷ�Χ��M_PI
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());
  // �����������ڵ��е� ������е���Ϊ���������
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}

// ���л��ڷ�֧�����㷨�Ĵ�ƥ��
bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);

  // Step: ��ԭ�㴦�ĵ�������ת��Ԥ��ķ�����
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));

  // Step: ���ɰ��ղ�ͬ�Ƕ���ת��ĵ��Ƽ���
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);

  // Step: ����ת��ĵ��Ƽ��ϰ���Ԥ�����ƽ��������ƽ��, ��ȡƽ�ƺ�ĵ��ڵ�ͼ�е�����
  // �������ɢ�����������ϸ�ķֱ��ʵĵ�ͼ����
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  
  // ��С�������ڵĴ�С, ����ÿһ֡�����ڱ�֤���һ�������ڵ�ͼ��Χ��ʱ������ƶ���Χ
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  // ������ͷֱ����е����еĺ�ѡ�� ��ͷֱ�����ͨ���������Ĳ�������ͼ�ķֱ��ʼ��������.
  // ���ڵ�ͼ����ϵ��˵ ��ͷֱ���=1<<h, h��ʾ���������ܵĲ���
  // ���ﲻ������ͷֱ��ʵ����к�ѡ��ĵ÷ֽ����˼���, ͬʱ�����մӴ�С����
  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  
  // Step: ���л��ڷ�֧�����㷨������, ��ȡ���Ž�
  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score); // param: max_depth
  
  // ������Ž��ֵ, �������ָ����ֵmin_score����Ϊƥ��ɹ�,������Ϊ��ƥ�䷵��ʧ��
  if (best_candidate.score > min_score) {
    *score = best_candidate.score;
    // Step: ���ݼ������ƫ������λ�˽���У׼
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

// ������ͷֱ��ʲ�(դ�����)�ϵ����к�ѡ��, �����д��������
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const {

  // ������ͷֱ��ʲ�(դ�����)�ϵ����к�ѡ��
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);

  // ����ÿ����ѡ��ĵ÷�, ����ƥ��÷ִӴ�С����, �������кõ�candidates 
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

// ������ͷֱ��ʲ�(դ�����)�ϵ����к�ѡ��
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  // ������ת���ÿ������
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {

    // X�����ѡ��ĸ���
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;

    // Y�����ѡ��ĸ���
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;

    // num_candidates Ϊ��ͷֱ�����һ�������к�ѡ����ܸ���
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }

  // �����к�ѡ�Ᵽ������, ��ѡ��ĽṹΪ���Ƕȵ�����, xƫ����, yƫ����, ����������
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);

  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        // ���ɺ�ѡ��, ����Ǻ�ѡ����ԭʼ����ԭ��������ƫ����
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

// �����еĺ�ѡ��������ֲ����н�������
void FastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  // �������еĺ�ѡ��, ��ÿ����ѡ����д��
  for (Candidate2D& candidate : *candidates) {
    int sum = 0;
    // xy_index Ϊ��֡��ת��ĵ����ϵ�ÿ�����Ӧ�ڵ�ͼ�ϵ�դ������
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      // ��ת��ĵ��Ƶ�ÿ������������������н��X��Y��ƫ��, �������ƽ���ƽ��
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);

      // ��ƽ�ƺ�ĵ��Ƶ�ÿ���� ��ȡ��precomputation_grid�϶�Ӧ��դ��ֵ
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }

    // դ��ֵ�ĺͳ�����������е�ĸ���, ��Ϊ�����ѡ������� precomputation_grid �ϵĵ÷�
    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }

  // ���ݺ�ѡ���score, �����к�ѡ����н�������
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());
}

/**
 * @brief ���ڶ�ֱ��ʵ�ͼ�ķ�֧���������㷨
 * 
 * @param[in] discrete_scans ������Ƶ�ÿ�����ڵ�ͼ�ϵ�դ������
 * @param[in] search_parameters �������ò���
 * @param[in] candidates ��ѡ��
 * @param[in] candidate_depth �������߶�
 * @param[in] min_score ��ѡ����С�÷�
 * @return Candidate2D ���Ž�
 */
Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float min_score) const {

  // ����������Եݹ���õķ�ʽ����
  // ���ȸ����˵ݹ���ֹ������, ����������˵�0��(������), ��ζ��������������һ��Ҷ�ӽڵ�.
  // ͬʱ����ÿ�ε����������Ƕ��Ƕ�����չ�ĺ�ѡ����н�������
  // ���Զ��׵����Ҷ�ӽڵ�������Ž�, ֱ�ӷ��ؼ���
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  // Ȼ�󴴽�һ����ʱ�ĺ�ѡ��, �����÷�����Ϊmin_score
  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;

  // �������еĺ�ѡ��
  for (const Candidate2D& candidate : candidates) {
    //  Step: ��֦ �������õ���ֵ ���� ������һ��Ŀ��н����߷� �Ŀ��нⲻ���м�����֦
    // �������һ����ѡ��ķֵ�����ֵ, ��ô��ߵĺ�ѡ��ĵ÷�Ҳ�������ֵ,�Ϳ���ֱ������ѭ����
    if (candidate.score <= min_score) {
      break;
    }

    // ���forѭ���ܹ���������, ˵����ǰ��ѡ����һ�����ŵ�ѡ��, ��Ҫ������з�֦
    std::vector<Candidate2D> higher_resolution_candidates;
    // ����������Ϊ�ϲ��һ��
    const int half_width = 1 << (candidate_depth - 1);

    // Step: ��֦ ��x��yƫ�ƽ��б���, ���candidate���ĸ��ӽڵ��ѡ��
    for (int x_offset : {0, half_width}) { // ֻ��ȡ0��half_width
      // ��������˽���, ������
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }

        // ��ѡ�������ƽ���, һ��4��,���Կ���, ��֦���緽���ķ�֦�������½ǵ��ĸ��ӽڵ���з�֦
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }

    // �������ɵ�4����ѡ����д��������, ͬһ������, ��ͬ��ͼ
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);

    // �ݹ����BranchAndBound�������ɵ�higher_resolution_candidates�������� 
    // �ȶ��������ߵĽڵ�������з�֧, ֱ����ײ�, Ȼ���ٷ��ص����ڶ����ٽ��е���
    // ��������ڶ������߷�û����һ������ײ㣨Ҷ�Ӳ㣩�ķ�����, ������, 
    // ����������½��з�֧������
 
    // Step: ���� best_high_resolution_candidate.score
    // �Ժ�ͨ���ݹ���÷����˸��ŵĽⶼ��ͨ��std::max������������֪�����Ž�.
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
