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

#ifndef CARTOGRAPHER_IO_COLOR_H_
#define CARTOGRAPHER_IO_COLOR_H_

#include <array>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace io {

using Uint8Color = std::array<uint8, 3>;
using FloatColor = std::array<float, 3>;

// A function for on-demand generation of a color palette, with every two
// direct successors having large contrast.
FloatColor GetColor(int id);
/**
 * @brief (0.0,1.0) --> (0,255) 线性变换
 * 
 * @param c 
 * @return uint8 
 */
inline uint8 FloatComponentToUint8(float c) {
  return static_cast<uint8>(cartographer::common::RoundToInt(
      cartographer::common::Clamp(c, 0.f, 1.f) * 255));
}
/**
 * @brief (0,255) --> (0.0,1.0) 线性变换
 * 
 * @param c 
 * @return float 
 */
inline float Uint8ComponentToFloat(uint8 c) { return c / 255.f; }
/**
 * @brief 颜色表示方式转换（float to uint8)
 * 
 * @param color 
 * @return Uint8Color 
 */
inline Uint8Color ToUint8Color(const FloatColor& color) {
  return {{FloatComponentToUint8(color[0]), FloatComponentToUint8(color[1]),
           FloatComponentToUint8(color[2])}};
}
/**
 * @brief 颜色表示方式转换 (unint8 to float)
 * 
 * @param color 
 * @return FloatColor 
 */
inline FloatColor ToFloatColor(const Uint8Color& color) {
  return {{Uint8ComponentToFloat(color[0]), Uint8ComponentToFloat(color[1]),
           Uint8ComponentToFloat(color[2])}};
}

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_COLOR_H_
