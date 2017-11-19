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

#ifndef CARTOGRAPHER_COMMON_MAKE_UNIQUE_H_
#define CARTOGRAPHER_COMMON_MAKE_UNIQUE_H_

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace cartographer {
namespace common {

// Implementation of c++14's std::make_unique, taken from
// https://isocpp.org/files/papers/N3656.txt
/**
 * @brief typename _Unique_if<T>::_Single_object make_unique()的辅助函数
 * 
 * @tparam T 
 */
template <class T>
struct _Unique_if {
  typedef std::unique_ptr<T> _Single_object;
};
/**
 * @brief typename _Unique_if<T[]>::_Single_object make_unique()的辅助函数
 * 
 * @tparam T 
 */
template <class T>
struct _Unique_if<T[]> {
  typedef std::unique_ptr<T[]> _Unknown_bound;
};
/**
 * @brief typename _Unique_if<T[N]>::_Single_object make_unique()的辅助函数
 * 
 * @tparam T 
 */
template <class T, size_t N>
struct _Unique_if<T[N]> {
  typedef void _Known_bound;
};
/**
 * @brief make_unique实现,使用方式:unique_ptr<string> p=common::make_unique<string>(6,'z');
 * 
 * @details 使用c++11的完美转发功能std::forward()
 *          请特别注意实现中的new T(std::forward<Args>(args)...)
 *          其中 std::forward<Args>(args)为展开模式,...为展开包
 *          该句可以展开为 new T(std::forward<Args>(arg1),std::forward<Args>(arg2),...)
 * 
 * @tparam T 
 * @tparam Args 
 */
template <class T, class... Args>
typename _Unique_if<T>::_Single_object make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
/**
 * @brief  make_unique实现,使用方式:unique_ptr<int[]> p=common::make_unique<int[]>(6);
 * 
 * @tparam T 
 */
template <class T>
typename _Unique_if<T>::_Unknown_bound make_unique(size_t n) {
  typedef typename std::remove_extent<T>::type U;
  return std::unique_ptr<T>(new U[n]());
}
/**
 * @brief 禁用
 * 
 * @tparam T 
 * @tparam Args 
 */
template <class T, class... Args>
typename _Unique_if<T>::_Known_bound make_unique(Args&&...) = delete;

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_MAKE_UNIQUE_H_
