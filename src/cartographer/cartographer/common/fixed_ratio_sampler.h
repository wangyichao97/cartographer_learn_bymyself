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

#ifndef CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_
#define CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_

#include <string>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

// Signals when a sample should be taken from a stream of data to select a
// uniformly distributed fraction of the data.
class FixedRatioSampler {
 public:
  // 显式构造函数，接受一个采样比例 ratio。这个比例表示希望采样的比例，例如0.1表示每10个数据中采样1个
  explicit FixedRatioSampler(double ratio);
  ~FixedRatioSampler();

  FixedRatioSampler(const FixedRatioSampler&) = delete;
  FixedRatioSampler& operator=(const FixedRatioSampler&) = delete;

  // Returns true if this pulse should result in an sample.
  bool Pulse();

  // Returns a debug string describing the current ratio of samples to pulses.
  std::string DebugString();

 private:
  // Sampling occurs if the proportion of samples to pulses drops below this
  // number.
  // 采样比例
  const double ratio_;
  // 脉冲计数器,记录调用 Pulse 方法的总次数，即处理的数据点数量
  int64 num_pulses_ = 0;
  // 样本计数器, 记录已经采样的数据点数量。
  int64 num_samples_ = 0;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_
