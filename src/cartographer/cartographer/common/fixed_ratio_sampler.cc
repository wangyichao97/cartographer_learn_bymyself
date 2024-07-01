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

#include "cartographer/common/fixed_ratio_sampler.h"

#include "glog/logging.h"

namespace cartographer {
namespace common {

FixedRatioSampler::FixedRatioSampler(const double ratio) : ratio_(ratio) {
  CHECK_GE(ratio, 0.);
  LOG_IF(WARNING, ratio == 0.) << "FixedRatioSampler is dropping all data.";
  CHECK_LE(ratio, 1.);
}

FixedRatioSampler::~FixedRatioSampler() {}

/**
 * @brief 这个方法决定了在调用时是否应该采样当前的数据。
 * 
 * @return true 表示当前数据点被采样
 * @return false 
 */
bool FixedRatioSampler::Pulse() {
  //  增加1，表示处理了一个新的数据点
  ++num_pulses_;
  // 计算当前的采样比例，即当前采样的样本数与脉冲总数之比, 如果当前采样比例小于设定的 ratio_，则表示需要进行采样。
  if (static_cast<double>(num_samples_) / num_pulses_ < ratio_) {
    // 如果需要采样，样本计数 num_samples_ 增加1
    ++num_samples_;
    return true;
  }
  return false;
}

std::string FixedRatioSampler::DebugString() {
  return std::to_string(num_samples_) + " (" +
         std::to_string(100. * num_samples_ / num_pulses_) + "%)";
}

}  // namespace common
}  // namespace cartographer
