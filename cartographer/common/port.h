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

#include <cinttypes>
#include <cmath>
#include <string>

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

using std::string;

namespace cartographer {
namespace common {

inline int RoundToInt(const float x) { return std::lround(x); }

/**
 * @brief 取整函数,最临近取整,0.5处正数向上取证,负数向下取证,e.g. -0.5->-1 0.5->1
 * 
 * @details 注意,若使用RoundToInt(x-0.5),则正数使用去尾法,负数使用进一法,0取-1,e.g. 1.8->1 -1.8->-2 0->-1
 * @param x 
 * @return int 
 */
inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return std::lround(x); }

inline int64 RoundToInt64(const double x) { return std::lround(x); }
/**
 * @brief 快速压缩字符串
 * 
 * @param uncompressed 
 * @param compressed 
 */
inline void FastGzipString(const string& uncompressed, string* compressed) {
  boost::iostreams::filtering_ostream out;
  out.push(
      boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
  out.push(boost::iostreams::back_inserter(*compressed));
  boost::iostreams::write(out,
                          reinterpret_cast<const char*>(uncompressed.data()),
                          uncompressed.size());
}
/**
 * @brief 快速解压字符串
 * 
 * @param compressed 
 * @param decompressed 
 */
inline void FastGunzipString(const string& compressed, string* decompressed) {
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_decompressor());
  out.push(boost::iostreams::back_inserter(*decompressed));
  boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()),
                          compressed.size());
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PORT_H_
