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

#include "cartographer/mapping/value_conversion_tables.h"

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr uint16 kUpdateMarker = 1u << 15;

/**
 * @brief ��[0, 1~32767] ӳ�䵽 [0.9, 0.1~0.9]
 * 
 * @param[in] value [0, 32767]��ֵ, 0 ��Ӧ0.9
 * @param[in] unknown_value 0
 * @param[in] unknown_result 0.9 
 * @param[in] lower_bound 0.1 �½�
 * @param[in] upper_bound 0.9 �Ͻ�
 * @return float ת�������ֵ
 */
float SlowValueToBoundedFloat(const uint16 value, 
                              const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_LE(value, 32767);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / 32766.f;
  return value * kScale + (lower_bound - kScale);
}

/**
 * @brief �½�ת����
 * 
 * @param[in] unknown_value 0 
 * @param[in] unknown_result 0.9 
 * @param[in] lower_bound 0.1 
 * @param[in] upper_bound 0.9 
 * @return std::unique_ptr<std::vector<float>> ת�����ָ��
 */
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = absl::make_unique<std::vector<float>>();
  // num_values = 65536
  size_t num_values = std::numeric_limits<uint16>::max() + 1; 
  // ����ռ�
  result->reserve(num_values);

  // ��[0, 1~32767]ӳ���[0.9, 0.1~0.9]
  // vector�ĸ���Ϊ65536, ���Դ����2��[0-32767]��ӳ��
  for (size_t value = 0; value != num_values; ++value) {
    result->push_back(SlowValueToBoundedFloat(
        static_cast<uint16>(value) & ~kUpdateMarker, // ȡ�ұ�15λ������, 0-32767
        unknown_value,
        unknown_result, lower_bound, upper_bound));
  }
  return result;
}
}  // namespace

/**
 * @brief ��ȡת����, �������ֻ�����1��
 * 
 * @param[in] unknown_result 0.9 δ֪ʱ��ֵ
 * @param[in] lower_bound 0.1 ��Сcorrespondence_cost
 * @param[in] upper_bound 0.9 ���correspondence_cost
 * @return const std::vector<float>* 
 */
const std::vector<float>* ValueConversionTables::GetConversionTable(
    float unknown_result, float lower_bound, float upper_bound) {
  // ��bounds��Ϊkey
  std::tuple<float, float, float> bounds =
      std::make_tuple(unknown_result, lower_bound, upper_bound);
  auto lookup_table_iterator = bounds_to_lookup_table_.find(bounds);

  // ���û��bounds���key���½�
  if (lookup_table_iterator == bounds_to_lookup_table_.end()) {
    // �½�ת����
    auto insertion_result = bounds_to_lookup_table_.emplace(
        bounds, PrecomputeValueToBoundedFloat(0, unknown_result, lower_bound,
                                              upper_bound));
    return insertion_result.first->second.get();
  } 
  // ������ھ�ֱ�ӷ���ԭʼָ��
  else {
    return lookup_table_iterator->second.get();
  }
}

}  // namespace mapping
}  // namespace cartographer
