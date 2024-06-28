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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H

#include <string>
#include <tuple>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer_ros/trajectory_options.h"

namespace cartographer_ros {

// Top-level options of Cartographer's ROS integration.
struct NodeOptions {
  // 用于配置地图构建器的选项
  ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
  // 地图坐标系
  std::string map_frame;
  // 查找坐标变换的超时时间，以秒为单位。设置为0.表示在查找变换时不等待。
  double lookup_transform_timeout_sec;
  // 子地图发布的时间周期，以秒为单位。
  double submap_publish_period_sec;
  // 位姿发布的时间周期，以秒为单位。
  double pose_publish_period_sec;
  // 轨迹发布的时间周期，以秒为单位。
  double trajectory_publish_period_sec;
  // 是否将位姿信息发布到TF（变换帧）树中。
  bool publish_to_tf = true;
  // 是否发布跟踪位姿
  bool publish_tracked_pose = false;
  // 是否使用位姿外推器
  bool use_pose_extrapolator = true;
};

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H
