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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

// 使用GFlags库来定义命令行标志，collect_metrics初始值为 false，使用时为./program --collect_metrics=true
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  // 构造 tf2_ros::Buffer 对象，设置缓存时间为 10 秒
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  //监听并缓存 tf上的独立线程
  tf2_ros::TransformListener tf(tf_buffer);

  NodeOptions node_options;
  TrajectoryOptions trajectory_options;

  // 根据lua配置文件的内容，为node_options和trajectory_options赋值
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  //创建了一个地图构建器 map_builder，根据节点选项中的地图构建器选项来选择合适的构建器
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  
  //node类初始化，将ros的topic传入slam
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);

  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  //单线程执行，其中的回调函数在任务队列中次序执行
  ::ros::spin();

  //结束所有处于活动状态的轨迹
  node.FinishAllTrajectories();
  //当所有轨迹结束后，再次执全局优化
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {

  // 初始化 Google's GLog 日志库
  google::InitGoogleLogging(argv[0]);
  // 使用GFlags 库解析命令行参数并将值存储在相应的全局变量中， true 表示如果在命令行中发现未知的标志中止程序
  google::ParseCommandLineFlags(&argc, &argv, true);

  //Glog库中的 CHECK 宏来进行运行时检查，查看这两个文件是不是为空
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  // 一般不需要显式调用，除非需要在创建nodehandle实例前启动ros相关程序
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
