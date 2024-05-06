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

#include "cartographer_ros/node_options.h"

#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "glog/logging.h"

namespace cartographer_ros {

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {

  NodeOptions options;

  //GetDictionary根据lua字典中的参数生成protobuf的序列化数据结构，传递给CreateMapBuilderOptions
  options.map_builder_options =
      ::cartographer::mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary->GetDictionary("map_builder").get());

  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.lookup_transform_timeout_sec =
      lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.submap_publish_period_sec =
      lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
  options.pose_publish_period_sec =
      lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
  options.trajectory_publish_period_sec =
      lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
  if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
    options.publish_to_tf =
        lua_parameter_dictionary->GetBool("publish_to_tf");
  }
  if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
    options.publish_tracked_pose =
        lua_parameter_dictionary->GetBool("publish_tracked_pose");
  }
  if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
    options.use_pose_extrapolator =
        lua_parameter_dictionary->GetBool("use_pose_extrapolator");
  }
  return options;
}

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
  // 获得配置文件所在的目录
  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  //读取配置文件内容到code中
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  // 根据给定的字符串，生成一个lua字典
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));
  // 将lua字典传播给这两个函数并返回
  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
}

}  // namespace cartographer_ros
