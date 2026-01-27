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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"
#include "cartographer/common/format_transform.h"

namespace cartographer {
namespace mapping {

PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
        const common::Duration pose_queue_duration,
        const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
    auto extrapolator = absl::make_unique<PoseExtrapolator>(
                pose_queue_duration, imu_gravity_time_constant);
    extrapolator->AddImuData(imu_data);
    extrapolator->imu_tracker_ =
            absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
    extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
                imu_data.linear_acceleration);
    extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
                imu_data.angular_velocity);
    extrapolator->imu_tracker_->Advance(imu_data.time);
    extrapolator->AddPose(
                imu_data.time,
                transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
    return extrapolator;
}

common::Time PoseExtrapolator::GetLastPoseTime() const {
    if (timed_pose_queue_.empty()) {
        return common::Time::min();
    }
    return timed_pose_queue_.back().time;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
    if (!extrapolation_imu_tracker_) {
        return common::Time::min();
    }
    return extrapolation_imu_tracker_->time();
}

//用于外界向位姿预测器更新最新位姿
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
    if (imu_tracker_ == nullptr) {
        common::Time tracker_start = time;
        if (!imu_data_.empty()) {
            tracker_start = std::min(tracker_start, imu_data_.front().time);
        }
        imu_tracker_ =
                absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
    }
    // LOG_INFO("pose2(%f, %f, %f), time2 = %fs", pose.translation().x(), pose.translation().y(),
    //          SlamCommon::RadToDeg(cartographer::transform::GetYaw(pose)),
    //          cartographer::common::ToSeconds(time.time_since_epoch()));
    //添加的时间和位姿入队
    timed_pose_queue_.push_back(TimedPose{time, pose});
    //剔除队列中比需要更新的位姿的时间戳早的数据，但是至少保证队列中有2帧数据
    while (timed_pose_queue_.size() > 2 &&
           timed_pose_queue_[1].time <= time - pose_queue_duration_) {
        timed_pose_queue_.pop_front();
    }
    UpdateVelocitiesFromPoses();
    AdvanceImuTracker(time, imu_tracker_.get());
    TrimImuData();
    TrimOdometryData();
    odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
    extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}

//将新采集的IMU数据放入数据队列，并剔除太老的数据
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
    CHECK(timed_pose_queue_.empty() ||
          imu_data.time >= timed_pose_queue_.back().time);
    // LOG_INFO("imu_acc: %f, %f, %f; imu_w: %f, %f, %f",
    //          imu_data.linear_acceleration.x(), imu_data.linear_acceleration.y(),
    //          imu_data.linear_acceleration.z(), imu_data.angular_velocity.x(),
    //          imu_data.angular_velocity.y(), imu_data.angular_velocity.z());
    // LOG_INFO("penghu* Q add imu data");
    imu_data_.push_back(imu_data);
    TrimImuData();
}

//将新采集的里程计数据放入数据队列，并剔除太老的数据
//同时，根据里程计位姿变化量，计算得到角速度和线速度
void PoseExtrapolator::AddOdometryData(
        const sensor::OdometryData& odometry_data) {
    CHECK(timed_pose_queue_.empty() ||
          odometry_data.time >= timed_pose_queue_.back().time);
    odometry_data_.push_back(odometry_data);
    // LOG(INFO)<<"614 FOR ODOM SIZE =" << odometry_data_.size();
    TrimOdometryData();
    if (odometry_data_.size() < 2) {
        // LOG_INFO("odom size < 2");
        return;
    }
    // TODO(whess): Improve by using more than just the last two odometry poses.
    // Compute extrapolation in the tracking frame.
    //const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
    //penghu*
    const sensor::OdometryData& odometry_data_oldest = odometry_data_[odometry_data_.size() - 2];
    const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
    // LOG_INFO("odometry_data_oldest(x = %f, y = %f)(w= %f, x= %f, y= %f, z= %f)", odometry_data_oldest.pose.translation().x(),
    //          odometry_data_oldest.pose.translation().y(),
    //          odometry_data_oldest.pose.rotation().w(),
    //          odometry_data_oldest.pose.rotation().x(),
    //          odometry_data_oldest.pose.rotation().y(),
    //          odometry_data_oldest.pose.rotation().z());
    // LOG_INFO("odometry_data_newest(x = %f, y = %f)(w= %f, x= %f, y= %f, z= %f)", odometry_data_newest.pose.translation().x(),
    //          odometry_data_newest.pose.translation().y(),
    //          odometry_data_newest.pose.rotation().w(),
    //          odometry_data_newest.pose.rotation().x(),
    //          odometry_data_newest.pose.rotation().y(),
    //          odometry_data_newest.pose.rotation().z());
    const double odometry_time_delta =
            common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
    const transform::Rigid3d odometry_pose_delta =
            odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
    // LOG_INFO("xuan zhuan piancha = %f", odometry_pose_delta.rotation().w());
    // LOG_INFO("(x= %f, y= %f)(w= %f, x= %f, y= %f, z= %f)", odometry_data_newest.pose.translation().x(),
    //          odometry_data_newest.pose.translation().y(),
    //          odometry_data_newest.pose.rotation().w(),
    //          odometry_data_newest.pose.rotation().x(),
    //          odometry_data_newest.pose.rotation().y(),
    //          odometry_data_newest.pose.rotation().z());
    // LOG_INFO("odometry_pose_delta(x= %f, y = %f)(w= %f, x= %f, y= %f, z= %f)", odometry_pose_delta.translation().x(),
    //          odometry_pose_delta.translation().y(),
    //          odometry_pose_delta.rotation().w(),
    //          odometry_pose_delta.rotation().x(),
    //          odometry_pose_delta.rotation().y(),
    //          odometry_pose_delta.rotation().z());

    //penghu*
    if(odometry_data_newest.pose.translation().x() == 0 && odometry_data_newest.pose.translation().y() &&
            odometry_data_newest.pose.rotation().z() == 0)
    {
        angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
    }else{
        angular_velocity_from_odometry_ =
                transform::RotationQuaternionToAngleAxisVector(
                    odometry_pose_delta.rotation()) /
                odometry_time_delta;
    }
//    LOG_INFO("xuan zhuan jiaosudu = (%f, %f, %f)",
//             angular_velocity_from_odometry_.x(),
//             angular_velocity_from_odometry_.y(),
//             angular_velocity_from_odometry_.z());
    if (timed_pose_queue_.empty()) {
        return;
    }
    const Eigen::Vector3d
            linear_velocity_in_tracking_frame_at_newest_odometry_time =
            odometry_pose_delta.translation() / odometry_time_delta;
    const Eigen::Quaterniond orientation_at_newest_odometry_time =
            timed_pose_queue_.back().pose.rotation() *
            ExtrapolateRotation(odometry_data_newest.time,
                                odometry_imu_tracker_.get());
    linear_velocity_from_odometry_ =
            orientation_at_newest_odometry_time *
            linear_velocity_in_tracking_frame_at_newest_odometry_time;
//     LOG_INFO("zhixian sudu = (%f, %f, %f)", linear_velocity_from_odometry_.x(),
//              linear_velocity_from_odometry_.y(),
//              linear_velocity_from_odometry_.z());
}

//外界调用位姿预测器进行位姿估计的接口
//将需要进行位姿估计的时间戳传入后，会根据时间戳插值预测出一个位姿
//要求传入的时间戳不能比所有已缓存的传感器数据老
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
//    LOG_INFO("newest_timed_pose(%f, %f, %f)", newest_timed_pose.pose.translation().x(),
//             newest_timed_pose.pose.translation().y(),
//             SlamCommon::RadToDeg(cartographer::transform::GetYaw(newest_timed_pose.pose)));
    CHECK_GE(time, newest_timed_pose.time);
    //如果这个时间戳的位姿已经预测过，则不需要重复预测了
    if (cached_extrapolated_pose_.time != time) {
        //预测平移量
        const Eigen::Vector3d translation =
                ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
        //预测旋转量
        const Eigen::Quaterniond rotation =
                newest_timed_pose.pose.rotation() *
                ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
        //缓存最近一次预测的位姿
        cached_extrapolated_pose_ =
                TimedPose{time, transform::Rigid3d{translation, rotation}};
    }
    return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
        const common::Time time) {
    ImuTracker imu_tracker = *imu_tracker_;
    AdvanceImuTracker(time, &imu_tracker);
    return imu_tracker.orientation();
}

//根据两个位姿来推算速度
void PoseExtrapolator::UpdateVelocitiesFromPoses() {
    if (timed_pose_queue_.size() < 2) {
        // We need two poses to estimate velocities.
        return;
    }
    CHECK(!timed_pose_queue_.empty());
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const auto newest_time = newest_timed_pose.time;
    const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
    const auto oldest_time = oldest_timed_pose.time;
    //时间变化量
    const double queue_delta = common::ToSeconds(newest_time - oldest_time);
    //推算速度的两个位姿的时间差不能太小，否则误差太大
    if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
        LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                     << queue_delta << " s";
        return;
    }
    const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
    const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
    //速度 = 位姿变量 / 时间差
    linear_velocity_from_poses_ =
            (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
    angular_velocity_from_poses_ =
            transform::RotationQuaternionToAngleAxisVector(
                oldest_pose.rotation().inverse() * newest_pose.rotation()) /
            queue_delta;
}

//裁剪IMU数据，IMU队列中只保留一帧比当前最新数据时间老的数据
//因为最新数据可能是里程计数据，这样IMU数据可能会被裁剪到只剩1帧，所以至少确保IMU数据还有2帧
void PoseExtrapolator::TrimImuData() {
    // 需要满足三个条件：IMU数据队列大于1，Pose的队列不为空，IMU数据队列的第一个元素时间小于Pose队列的最后一个元素的时间
    // 最后一个条件意味着当IMU数据的时间比一个最新的Pose的时间要早时，说明这个IMU数据已经过期了。所以从队列中删掉就可以了。
    // 知道IMU数据的时间要比最新的Pose时间晚，那么说明这时候这个数据还有用。
    // 这种情况就不再删了，跳出循环，等待其他程序取出队列最开头的IMU数据进行融合
    while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
           imu_data_[1].time <= timed_pose_queue_.back().time) {
        imu_data_.pop_front();
    }
}

//裁剪里程计数据，里程计队列中只保留一帧比当前最新数据时间老的数据
//因为最新数据可能是IMU数据，这样里程计数据可能会被裁剪到只剩1帧，所以至少确保里程计数据还有2帧
void PoseExtrapolator::TrimOdometryData() {
    while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
           odometry_data_[1].time <= timed_pose_queue_.back().time) {
        odometry_data_.pop_front();
    }
}

//更新IMU跟踪器的内部数据
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
    CHECK_GE(time, imu_tracker->time());
    if (imu_data_.empty() || time < imu_data_.front().time) {
        // There is no IMU data until 'time', so we advance the ImuTracker and use
        // the angular velocities from poses and fake gravity to help 2D stability.
        //在“time”之前没有IMU数据，则使用来自位姿和假重力的角速度让ImuTracker预测位姿，以提升2D的稳定性。
        imu_tracker->Advance(time);
        imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
//        LOG_INFO("odometry_data_size = %d", odometry_data_.size());
        imu_tracker->AddImuAngularVelocityObservation(
/*                    odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                              : */angular_velocity_from_odometry_);
        return;
    }
    // LOG_INFO("penghu* Q AdvanceImuTracker");
    if (imu_tracker->time() < imu_data_.front().time) {
        // Advance to the beginning of 'imu_data_'.
        imu_tracker->Advance(imu_data_.front().time);
    }
    auto it = std::lower_bound(
                imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
                [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
    });
    while (it != imu_data_.end() && it->time < time) {
        imu_tracker->Advance(it->time);
        imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
        imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
        ++it;
    }
    imu_tracker->Advance(time);
}

//预测旋转预变化量
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
        const common::Time time, ImuTracker* const imu_tracker) const {
    CHECK_GE(time, imu_tracker->time());
    AdvanceImuTracker(time, imu_tracker);
    const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
    return last_orientation.inverse() * imu_tracker->orientation();
}

//预测平移变化量，平移变化量 = 时间差 * 速度（以里程计的速度为准）
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const double extrapolation_delta =
            common::ToSeconds(time - newest_timed_pose.time);
    if (odometry_data_.size() < 2) {
        return extrapolation_delta * linear_velocity_from_poses_;
    }
    return extrapolation_delta * linear_velocity_from_odometry_;
}

PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
        const std::vector<common::Time>& times) {
    std::vector<transform::Rigid3f> poses;
    for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
        poses.push_back(ExtrapolatePose(*it).cast<float>());
    }

    const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
            ? linear_velocity_from_poses_
            : linear_velocity_from_odometry_;
    return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                current_velocity,
                EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer
