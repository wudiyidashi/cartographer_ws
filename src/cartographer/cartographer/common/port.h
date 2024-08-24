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

#ifndef CARTOGRAPHER_COMMON_PORT_H_
#define CARTOGRAPHER_COMMON_PORT_H_

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cinttypes>
#include <cmath>
#include <string>

namespace cartographer {

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;  // 2���ֽڵ��޷�������
using uint32 = uint32_t;
using uint64 = uint64_t;  // 8���ֽڵ��޷�������

namespace common {

// c++11: std::lround ������ӽ�x��long int���� eg: lround(15.2) -> 15, lround(15.8) -> 16
// ������ӽ�x������
inline int RoundToInt(const float x) { return std::lround(x); }

inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return std::lround(x); }

inline int64 RoundToInt64(const double x) { return std::lround(x); }

/**
 * @brief ���ַ�������ѹ��
 * 
 * @param[in] uncompressed ѹ��ǰ��string
 * @param[out] compressed ѹ�����string
 */
inline void FastGzipString(const std::string& uncompressed,
                           std::string* compressed) {
  boost::iostreams::filtering_ostream out;
  // ��gzipѹ��
  out.push(
      boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
  out.push(boost::iostreams::back_inserter(*compressed));
  boost::iostreams::write(out,
                          reinterpret_cast<const char*>(uncompressed.data()),
                          uncompressed.size());
}

/**
 * @brief ���ַ������н�ѹ
 * 
 * @param[in] compressed ѹ����string
 * @param[out] decompressed ��ѹ���string
 */
inline void FastGunzipString(const std::string& compressed,
                             std::string* decompressed) {
  boost::iostreams::filtering_ostream out;
  // ��gzip��ѹ
  out.push(boost::iostreams::gzip_decompressor());
  out.push(boost::iostreams::back_inserter(*decompressed));
  boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()),
                          compressed.size());
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PORT_H_
