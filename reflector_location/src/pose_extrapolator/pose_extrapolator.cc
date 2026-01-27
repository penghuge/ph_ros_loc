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

#include "pose_extrapolator/pose_extrapolator.h"
#include <algorithm>
#include "absl/memory/memory.h"

namespace cartographer {
namespace mapping {

PoseExtrapolator::PoseExtrapolator(const ros::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant) {
    geometry_msgs::Pose pose;

    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), quat);

    pose.position.x = 0.0; pose.position.y = 0.0; pose.position.z = 0.0;
    pose.orientation = quat;
    cached_extrapolated_pose_ = {ros::Time::MIN, pose};
}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
        const ros::Duration pose_queue_duration,
        const double imu_gravity_time_constant, const ImuData& imu_data) {
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

    geometry_msgs::Pose pose;
    geometry_msgs::Quaternion quat;

    double yaw = extrapolator->imu_tracker_->orientation().normalized().toRotationMatrix().eulerAngles(0, 1, 2)[2];
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), quat);

    pose.position.x = 0.0; pose.position.y = 0.0; pose.position.z = 0.0;
    pose.orientation = quat;

    extrapolator->AddPose(imu_data.time, pose);
    return extrapolator;
}

ros::Time PoseExtrapolator::GetLastPoseTime() const {
    if (timed_pose_queue_.empty()) {
        return ros::Time::MIN;
    }
    return timed_pose_queue_.back().time;
}

ros::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
    if (!extrapolation_imu_tracker_) {
        return ros::Time::MIN;
    }
    return extrapolation_imu_tracker_->time();
}

//用于外界向位姿预测器更新最新位姿
void PoseExtrapolator::AddPose(const ros::Time time,
                               const geometry_msgs::Pose& pose) {
    if (imu_tracker_ == nullptr) {
        ros::Time tracker_start = time;
        if (!imu_data_.empty()) {
            tracker_start = std::min(tracker_start, imu_data_.front().time);
        }
        imu_tracker_ =
                absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
    }

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
void PoseExtrapolator::AddImuData(const ImuData& imu_data) {
    if(timed_pose_queue_.empty() && imu_data.time >= timed_pose_queue_.back().time)
        return;

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
        const OdometryData& odometry_data) {
    if(timed_pose_queue_.empty())
        return;
    if(odometry_data.time >= timed_pose_queue_.back().time)
        return;
    odometry_data_.push_back(odometry_data);

    TrimOdometryData();
    if (odometry_data_.size() < 2) {
        return;
    }
    // TODO(whess): Improve by using more than just the last two odometry poses.
    // Compute extrapolation in the tracking frame.
    //const OdometryData& odometry_data_oldest = odometry_data_.front();
    const OdometryData& odometry_data_oldest = odometry_data_[odometry_data_.size() - 2];
    const OdometryData& odometry_data_newest = odometry_data_.back();

    const double odometry_time_delta =
            ros::Duration(odometry_data_oldest.time - odometry_data_newest.time).toSec();

    Eigen::Quaterniond newest_quat(odometry_data_newest.pose.orientation.w, odometry_data_newest.pose.orientation.x,
        odometry_data_newest.pose.orientation.y, odometry_data_newest.pose.orientation.z);
    Eigen::Quaterniond oldest_quat(odometry_data_oldest.pose.orientation.w, odometry_data_oldest.pose.orientation.x,
        odometry_data_oldest.pose.orientation.y, odometry_data_oldest.pose.orientation.z);

    if(odometry_data_newest.pose.position.x == 0 && odometry_data_newest.pose.position.y &&
            odometry_data_newest.pose.position.z == 0){
        angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
    }else{
        angular_velocity_from_odometry_ = RotationQuaternionToAngleAxisVector(
            newest_quat.inverse() * oldest_quat) /
                odometry_time_delta;
    }

    if (timed_pose_queue_.empty()) {
        return;
    }

    Eigen::Vector3d newest_pose_translation(odometry_data_newest.pose.position.x, odometry_data_newest.pose.position.y, odometry_data_newest.pose.position.z);
    Eigen::Vector3d oldest_pose_translation(odometry_data_oldest.pose.position.x, odometry_data_oldest.pose.position.y, odometry_data_oldest.pose.position.z);
    Eigen::Vector3d odometry_translation_delta = oldest_pose_translation - newest_pose_translation;
    const Eigen::Vector3d
            linear_velocity_in_tracking_frame_at_newest_odometry_time = odometry_translation_delta / odometry_time_delta;

    TimedPose current_pose = timed_pose_queue_.back();
    Eigen::Quaterniond current_quat(current_pose.pose.orientation.w, current_pose.pose.orientation.x,
        current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    const Eigen::Quaterniond orientation_at_newest_odometry_time = current_quat *
            ExtrapolateRotation(odometry_data_newest.time,
                                odometry_imu_tracker_.get());
    linear_velocity_from_odometry_ =
            orientation_at_newest_odometry_time *
            linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

//外界调用位姿预测器进行位姿估计的接口
//将需要进行位姿估计的时间戳传入后，会根据时间戳插值预测出一个位姿
//要求传入的时间戳不能比所有已缓存的传感器数据老
geometry_msgs::Pose PoseExtrapolator::ExtrapolatePose(const ros::Time time) {
    if(timed_pose_queue_.empty()){
        geometry_msgs::Pose pose;
        geometry_msgs::Quaternion quat;
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), quat);
        pose.position.x = 0.0; pose.position.y = 0.0; pose.position.z = 0.0;
        pose.orientation = quat;
        return pose;
    }
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
//    LOG_INFO("newest_timed_pose(%f, %f, %f)", newest_timed_pose.pose.translation().x(),
//             newest_timed_pose.pose.translation().y(),
//             SlamCommon::RadToDeg(cartographer::transform::GetYaw(newest_timed_pose.pose)));
    if(time < newest_timed_pose.time)
        return newest_timed_pose.pose;

    //如果这个时间戳的位姿已经预测过，则不需要重复预测了
    if (cached_extrapolated_pose_.time != time) {
        //预测平移量
        const Eigen::Vector3d translation =
                ExtrapolateTranslation(time) + Eigen::Vector3d(newest_timed_pose.pose.position.x,
                                                               newest_timed_pose.pose.position.y,
                                                               newest_timed_pose.pose.position.z);
        //预测旋转量
        const Eigen::Quaterniond rotation =
                Eigen::Quaterniond(newest_timed_pose.pose.orientation.w, 
                                    newest_timed_pose.pose.orientation.x, 
                                    newest_timed_pose.pose.orientation.y, 
                                    newest_timed_pose.pose.orientation.z) *
                ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
        //缓存最近一次预测的位姿
        geometry_msgs::Pose tmp_pose;
        tmp_pose.position.x = translation.x();
        tmp_pose.position.y = translation.y();
        tmp_pose.position.z = translation.z();
        tmp_pose.orientation.w = rotation.w();
        tmp_pose.orientation.x = rotation.x();
        tmp_pose.orientation.y = rotation.y();
        tmp_pose.orientation.z = rotation.z();
        cached_extrapolated_pose_ = TimedPose{time, tmp_pose};
    }
    return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
        const ros::Time time) {
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
    if(timed_pose_queue_.empty()) 
        return;
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const auto newest_time = newest_timed_pose.time;
    const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
    const auto oldest_time = oldest_timed_pose.time;
    //时间变化量
    const double queue_delta = ros::Duration(newest_time - oldest_time).toSec();
    //推算速度的两个位姿的时间差不能太小，否则误差太大
    if (queue_delta < pose_queue_duration_.toSec()) {
        ROS_WARN("Queue too short for velocity estimation. Queue duration: %f s", queue_delta);

        return;
    }
    const geometry_msgs::Pose& newest_pose = newest_timed_pose.pose;
    const geometry_msgs::Pose& oldest_pose = oldest_timed_pose.pose;
    Eigen::Vector3d newest_pose_translation(newest_pose.position.x, newest_pose.position.y, newest_pose.position.z);
    Eigen::Vector3d oldest_pose_translation(oldest_pose.position.x, oldest_pose.position.y, oldest_pose.position.z);

    Eigen::Quaterniond newest_quat(newest_pose.orientation.w, newest_pose.orientation.x,
                                        newest_pose.orientation.y, newest_pose.orientation.z);
    Eigen::Quaterniond oldest_quat(oldest_pose.orientation.w, oldest_pose.orientation.x,
        oldest_pose.orientation.y, oldest_pose.orientation.z);
    //速度 = 位姿变量 / 时间差
    linear_velocity_from_poses_ =
            (newest_pose_translation - oldest_pose_translation) / queue_delta;
    angular_velocity_from_poses_ = RotationQuaternionToAngleAxisVector(
                oldest_quat.inverse() * newest_quat) /
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
void PoseExtrapolator::AdvanceImuTracker(const ros::Time time,
                                         ImuTracker* const imu_tracker) const {
    if(time < imu_tracker->time()) return;
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
                [](const ImuData& imu_data, const ros::Time& time) {
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
        const ros::Time time, ImuTracker* const imu_tracker) const {
    if(time < imu_tracker->time())
        return Eigen::Quaterniond::Identity();
        
    AdvanceImuTracker(time, imu_tracker);
    const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
    return last_orientation.inverse() * imu_tracker->orientation();
}

//预测平移变化量，平移变化量 = 时间差 * 速度（以里程计的速度为准）
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(ros::Time time) {
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const double extrapolation_delta =
            ros::Duration(time - newest_timed_pose.time).toSec();
    if (odometry_data_.size() < 2) {
        return extrapolation_delta * linear_velocity_from_poses_;
    }
    return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace mapping
}  // namespace cartographer
