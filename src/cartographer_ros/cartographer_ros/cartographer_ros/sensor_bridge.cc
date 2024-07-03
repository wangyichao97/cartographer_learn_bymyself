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

#include "cartographer_ros/sensor_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const bool ignore_out_of_order_messages, const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      ignore_out_of_order_messages_(ignore_out_of_order_messages),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) {}

std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
    const nav_msgs::Odometry::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  return absl::make_unique<carto::sensor::OdometryData>(
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
}

// 如果消息的时间戳早于或等于上一次处理的消息时间戳，并且配置了忽略乱序消息，则该消息将被忽略
bool SensorBridge:: IgnoreMessage(const std::string& sensor_id,
                                 cartographer::common::Time sensor_time) {
  if (!ignore_out_of_order_messages_) {
    return false;
  }
  auto it = latest_sensor_time_.find(sensor_id);
  if (it == latest_sensor_time_.end()) {
    return false;
  }
  return sensor_time <= it->second;
}

void SensorBridge::HandleOdometryMessage(
    const std::string& sensor_id, const nav_msgs::Odometry::ConstPtr& msg) 
{
  // 转换里程计消息
  std::unique_ptr<carto::sensor::OdometryData> odometry_data = ToOdometryData(msg);
  // 检查数据有效性
  if (odometry_data != nullptr) 
  {
    if (IgnoreMessage(sensor_id, odometry_data->time)) {
      LOG(WARNING) << "Ignored odometry message from sensor " << sensor_id
                   << " because sensor time " << odometry_data->time
                   << " is not before last odometry message time "
                   << latest_sensor_time_[sensor_id];
      return;
    }
    // 更新最后一次接收消息的时间
    latest_sensor_time_[sensor_id] = odometry_data->time;
    // 添加里程计数据到轨迹构建器
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}

void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id, const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
  // 时间戳转换
  const carto::common::Time time = FromRos(msg->header.stamp);
  // 无定位时
  if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::FixedFramePoseData{time, absl::optional<Rigid3d>()});
    return;
  }
  //坐标转换初始化：
  // 如果尚未计算本地坐标系的转换（ecef_to_local_frame_），则根据定位消息的经纬度计算出本地坐标系。
  if (!ecef_to_local_frame_.has_value()) {
    ecef_to_local_frame_ =
        ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << msg->latitude << ", long = " << msg->longitude << ".";
  }

  /**
   * @brief 将处理过的定位数据添加到轨迹构建器中
   * 
   */
  trajectory_builder_->AddSensorData(
      sensor_id, carto::sensor::FixedFramePoseData{
                     time, 
                     // 定位数据有效，就会创建一个包含姿态信息的 Rigid3d 对象。
                     absl::optional<Rigid3d>(Rigid3d::Translation(
                               ecef_to_local_frame_.value() *
                               LatLongAltToEcef(msg->latitude, msg->longitude,
                                                msg->altitude)))});
}

void SensorBridge::HandleLandmarkMessage(
    const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) 
{
  // 地标消息转换为 landmark_data 结构体
  auto landmark_data = ToLandmarkData(*msg);

  // 查找传感器到跟踪坐标系的变换
  auto tracking_from_landmark_sensor = tf_bridge_.LookupToTracking(
      landmark_data.time, CheckNoLeadingSlash(msg->header.frame_id));

  if (tracking_from_landmark_sensor != nullptr) 
  {
    // 转换地标到跟踪坐标系下
    for (auto& observation : landmark_data.landmark_observations) 
    {
      observation.landmark_to_tracking_transform =
          *tracking_from_landmark_sensor *
          observation.landmark_to_tracking_transform;
    }
  }
  trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}

std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg) 
{
  // 检查 IMU 数据的有效性
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  // 时间戳转换为Cartographer 使用的 carto::common::Time 类型。
  const carto::common::Time time = FromRos(msg->header.stamp);
  // 查找传感器到跟踪坐标系的变换
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";
  // 创建并返回 ImuData 对象
  return absl::make_unique<carto::sensor::ImuData>(carto::sensor::ImuData{  //构造对象
      time, // 时间戳
      sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration), //将线性加速度从传感器坐标系转换到跟踪坐标系
      sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});  // 将角速度从传感器坐标系转换到跟踪坐标系
}

void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::Imu::ConstPtr& msg) 
{
  // 将 ROS IMU 消息转换为 Cartographer IMU 数据
  std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
  if (imu_data != nullptr) {
    if (IgnoreMessage(sensor_id, imu_data->time)) {
      LOG(WARNING) << "Ignored IMU message from sensor " << sensor_id
                   << " because sensor time " << imu_data->time
                   << " is not before last IMU message time "
                   << latest_sensor_time_[sensor_id];
      return;
    }
    // 更新最新的传感器时间
    latest_sensor_time_[sensor_id] = imu_data->time;
    // 添加 IMU 数据到轨迹构建器
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time, 
                               imu_data->linear_acceleration,
                               imu_data->angular_velocity});
  }
}

// 将激光扫描消息转换为带有强度信息的点云，并调用另一个处理方法 HandleLaserScan
void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) 
{
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  // std::tie：解构赋值，将转换结果赋值给 point_cloud 和 time
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleRangefinder(sensor_id, time, msg->header.frame_id, point_cloud.points);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

// 需要将一帧激光扫描数据进行分段，以便更细粒度地处理每个分段的数据
void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) 
{
  // 检查点云是否为空
  if (points.points.empty()) {
    return;
  }
  // 检查最后一个点的时间是否小于等于0
  CHECK_LE(points.points.back().time, 0.f);
  // TODO(gaschler): Use per-point time instead of subdivisions.
  // 分段处理激光扫描数据
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    // 表示当前分段的点云数据。
    carto::sensor::TimedPointCloud subdivision(
        points.points.begin() + start_index, points.points.begin() + end_index);
    // 检查分段是否为空
    if (start_index == end_index) {
      continue;
    }
    // 当前分段的最后一个点的时间
    const double time_to_subdivision_end = subdivision.back().time;
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    // 表示当前分段的结束时间
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);

    /**
     * @brief 检查当前传感器的上一个分段时间是否在当前分段时间之前，如果不是，则忽略当前分段。
     */
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    // 更新传感器的上一个分段时间为当前分段时间。
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    // 调整每个点的相对时间，使最后一个点的时间为0
    for (auto& point : subdivision) {
      point.time -= time_to_subdivision_end;
    }
    // 确保分段中的最后一个点的时间确实为0。
    CHECK_EQ(subdivision.back().time, 0.f);
    
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  }
}

void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) 
{
  if (!ranges.empty()) {
    CHECK_LE(ranges.back().time, 0.f);
  }

  // 查找传感器到跟踪坐标系的变换
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  if (sensor_to_tracking != nullptr) 
  {
    //忽略重复的消息
    if (IgnoreMessage(sensor_id, time)) {
      LOG(WARNING) << "Ignored Rangefinder message from sensor " << sensor_id
                   << " because sensor time " << time
                   << " is not before last Rangefinder message time "
                   << latest_sensor_time_[sensor_id];
      return;
    }
    // 更新最新传感器时间
    latest_sensor_time_[sensor_id] = time;
    // 添加传感器数据到轨迹构建器
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       time, sensor_to_tracking->translation().cast<float>(),
                       carto::sensor::TransformTimedPointCloud(
                           ranges, sensor_to_tracking->cast<float>())});
  }
}

}  // namespace cartographer_ros
