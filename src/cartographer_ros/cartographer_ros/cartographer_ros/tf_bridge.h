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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TF_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TF_BRIDGE_H

#include <memory>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/time_conversion.h"
#include "tf2_ros/buffer.h"

namespace cartographer_ros {

class TfBridge {
 public:
 /**
  * @brief Construct a new Tf Bridge object
  * 
  * @param tracking_frame 要转换到的跟踪帧
  * @param lookup_transform_timeout_sec 查找变换的超时时间（以秒为单位）
  * @param buffer 指向 tf2_ros::Buffer 对象的指针，用于查找变换
  */
  TfBridge(const std::string& tracking_frame,
           double lookup_transform_timeout_sec, const tf2_ros::Buffer* buffer);
  ~TfBridge() {}

  TfBridge(const TfBridge&) = delete;
  TfBridge& operator=(const TfBridge&) = delete;

  // Returns the transform for 'frame_id' to 'tracking_frame_' if it exists at
  // 'time'.
  std::unique_ptr<::cartographer::transform::Rigid3d> LookupToTracking(
      ::cartographer::common::Time time, const std::string& frame_id) const;

 private:
  // 要转换到的跟踪帧
  const std::string tracking_frame_;
  // 查找变换的超时时间
  const double lookup_transform_timeout_sec_;
  // 指向 tf2_ros::Buffer 对象的指针，用于查找变换
  const tf2_ros::Buffer* const buffer_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TF_BRIDGE_H
