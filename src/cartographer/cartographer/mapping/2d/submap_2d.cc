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

#include "cartographer/mapping/2d/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_range_data_inserter_2d.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// submap2d�Ĳ�����������
proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SubmapsOptions2D options;
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_grid_options_2d() = CreateGridOptions2D(
      parameter_dictionary->GetDictionary("grid_options_2d").get());
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());

  bool valid_range_data_inserter_grid_combination = false;
  // ��ͼ����
  const proto::GridOptions2D_GridType& grid_type =
      options.grid_options_2d().grid_type();
  // ��scanд�ɵ�ͼ�ķ�ʽ
  const proto::RangeDataInserterOptions_RangeDataInserterType&
      range_data_inserter_type =
          options.range_data_inserter_options().range_data_inserter_type();
  if (grid_type == proto::GridOptions2D::PROBABILITY_GRID &&
      range_data_inserter_type ==
          proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) {
    valid_range_data_inserter_grid_combination = true;
  }
  if (grid_type == proto::GridOptions2D::TSDF &&
      range_data_inserter_type ==
          proto::RangeDataInserterOptions::TSDF_INSERTER_2D) {
    valid_range_data_inserter_grid_combination = true;
  }
  CHECK(valid_range_data_inserter_grid_combination)
      << "Invalid combination grid_type " << grid_type
      << " with range_data_inserter_type " << range_data_inserter_type;
  CHECK_GT(options.num_range_data(), 0);
  return options;
}

/**
 * @brief ���캯��
 * 
 * @param[in] origin Submap2D��ԭ��,������Submap����
 * @param[in] grid ��ͼ���ݵ�ָ��
 * @param[in] conversion_tables ��ͼ���ݵ�ת����
 */
Submap2D::Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      conversion_tables_(conversion_tables) {
  grid_ = std::move(grid);
}

// ����proto::Submap��ʽ����������Submap2D
Submap2D::Submap2D(const proto::Submap2D& proto,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::ToRigid3(proto.local_pose())),
      conversion_tables_(conversion_tables) {
  if (proto.has_grid()) {
    if (proto.grid().has_probability_grid_2d()) {
      grid_ =
          absl::make_unique<ProbabilityGrid>(proto.grid(), conversion_tables_);
    } else if (proto.grid().has_tsdf_2d()) {
      grid_ = absl::make_unique<TSDF2D>(proto.grid(), conversion_tables_);
    } else {
      LOG(FATAL) << "proto::Submap2D has grid with unknown type.";
    }
  }
  set_num_range_data(proto.num_range_data());
  set_insertion_finished(proto.finished());
}

// ����mapping::Submap2D����proto::Submap��ʽ������
proto::Submap Submap2D::ToProto(const bool include_grid_data) const {
  proto::Submap proto;
  auto* const submap_2d = proto.mutable_submap_2d();
  *submap_2d->mutable_local_pose() = transform::ToProto(local_pose());
  submap_2d->set_num_range_data(num_range_data());
  submap_2d->set_finished(insertion_finished());
  if (include_grid_data) {
    CHECK(grid_);
    *submap_2d->mutable_grid() = grid_->ToProto();
  }
  return proto;
}

// ����proto::Submap��ʽ�����ݸ��µ�ͼ
void Submap2D::UpdateFromProto(const proto::Submap& proto) {
  CHECK(proto.has_submap_2d());
  const auto& submap_2d = proto.submap_2d();
  set_num_range_data(submap_2d.num_range_data());
  set_insertion_finished(submap_2d.finished());
  if (proto.submap_2d().has_grid()) {
    if (proto.submap_2d().grid().has_probability_grid_2d()) {
      grid_ = absl::make_unique<ProbabilityGrid>(proto.submap_2d().grid(),
                                                 conversion_tables_);
    } else if (proto.submap_2d().grid().has_tsdf_2d()) {
      grid_ = absl::make_unique<TSDF2D>(proto.submap_2d().grid(),
                                        conversion_tables_);
    } else {
      LOG(FATAL) << "proto::Submap2D has grid with unknown type.";
    }
  }
}

/**
 * @brief ����ͼ����ѹ��, ����response
 * 
 * @param[out] response ѹ����ĵ�ͼ����
 */
void Submap2D::ToResponseProto(
    const transform::Rigid3d&,
    proto::SubmapQuery::Response* const response) const {
  if (!grid_) return;
  response->set_submap_version(num_range_data());
  // note: const��*���, ָ��ָ��ĵ�ַ���ܱ�,���ڴ浥Ԫ�е����ݿɱ�
  proto::SubmapQuery::Response::SubmapTexture* const texture =
      response->add_textures();
  // ���ѹ���������
  grid()->DrawToSubmapTexture(texture, local_pose());
}

// ���״�����д��դ���ͼ��
void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const RangeDataInserterInterface* range_data_inserter) {
  CHECK(grid_);
  CHECK(!insertion_finished());
  // ���״�����д��դ���ͼ��
  range_data_inserter->Insert(range_data, grid_.get());
  // ���뵽��ͼ�е��״����ݵĸ�����1
  set_num_range_data(num_range_data() + 1);
}

// ����ͼ���Ϊ���״̬
void Submap2D::Finish() {
  CHECK(grid_);
  CHECK(!insertion_finished());
  grid_ = grid_->ComputeCroppedGrid();
  // ����ͼ���Ϊ���״̬
  set_insertion_finished(true);
}

/********** ActiveSubmaps2D *****************/

// ActiveSubmaps2D���캯��
ActiveSubmaps2D::ActiveSubmaps2D(const proto::SubmapsOptions2D& options)
    : options_(options), range_data_inserter_(CreateRangeDataInserter()) {}

// ����ָ�� Submap2D �� shared_ptrָ�� ��vector
std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const {
  return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(),
                                                      submaps_.end());
}

// ����������д�뵽submap��
std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
    const sensor::RangeData& range_data) {
  // ����ڶ�����ͼ����ڵ�����ݵ���num_range_dataʱ,���½�����ͼ
  // ��Ϊ��ʱ��һ����ͼӦ���Ѿ��������״̬��
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());
  }
  // ��һ֡�״�����ͬʱд��������ͼ��
  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_.get());
  }
  // ��һ����ͼ�Ľڵ���������2����num_range_dataʱ,�ڶ�����ͼ�ڵ�����Ӧ�õ���num_range_data
  if (submaps_.front()->num_range_data() == 2 * options_.num_range_data()) {
    submaps_.front()->Finish();
  }
  return submaps();
}

// ������ͼ����д����
std::unique_ptr<RangeDataInserterInterface>
ActiveSubmaps2D::CreateRangeDataInserter() {
  switch (options_.range_data_inserter_options().range_data_inserter_type()) {
    // ����դ���ͼ��д����
    case proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D:
      return absl::make_unique<ProbabilityGridRangeDataInserter2D>(
          options_.range_data_inserter_options()
              .probability_grid_range_data_inserter_options_2d());
    // tsdf��ͼ��д����
    case proto::RangeDataInserterOptions::TSDF_INSERTER_2D:
      return absl::make_unique<TSDFRangeDataInserter2D>(
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d());
    default:
      LOG(FATAL) << "Unknown RangeDataInserterType.";
  }
}

// �Ե�ǰ�״�ԭ��Ϊ��ͼԭ��������ͼ
std::unique_ptr<GridInterface> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin) {
  // ��ͼ��ʼ��С,100��դ��
  constexpr int kInitialSubmapSize = 100;
  float resolution = options_.grid_options_2d().resolution(); // param: grid_options_2d.resolution
  switch (options_.grid_options_2d().grid_type()) {
    // ����դ���ͼ
    case proto::GridOptions2D::PROBABILITY_GRID:
      return absl::make_unique<ProbabilityGrid>(
          MapLimits(resolution,
                    // ���Ͻ�����Ϊ����ϵ�����ֵ, originλ�ڵ�ͼ���м�
                    origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                                resolution *
                                                Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
          &conversion_tables_);
    // tsdf��ͼ
    case proto::GridOptions2D::TSDF:
      return absl::make_unique<TSDF2D>(
          MapLimits(resolution,
                    origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                                resolution *
                                                Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d()
              .truncation_distance(),               // 0.3
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d()
              .maximum_weight(),                    // 10.0
          &conversion_tables_);
    default:
      LOG(FATAL) << "Unknown GridType.";
  }
}

// ����һ����ͼ,������ͼ�����ж��Ƿ�ɾ����һ����ͼ
void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  // ����AddSubmapʱ��һ����ͼһ�������״̬,������ͼ��Ϊ2ʱ�Ϳ���ɾ����һ����ͼ��
  if (submaps_.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(submaps_.front()->insertion_finished());
    // ɾ����һ����ͼ��ָ��
    submaps_.erase(submaps_.begin());
  }
  // �½�һ����ͼ, ������ָ������ͼ������ָ��
  submaps_.push_back(absl::make_unique<Submap2D>(
      origin,
      std::unique_ptr<Grid2D>(
          static_cast<Grid2D*>(CreateGrid(origin).release())),
      &conversion_tables_));
}

}  // namespace mapping
}  // namespace cartographer
