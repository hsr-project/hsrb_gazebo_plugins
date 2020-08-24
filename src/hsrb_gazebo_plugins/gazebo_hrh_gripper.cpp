/// vim: fileencoding=utf-8 :
/// @file gazebo_hrh_gripper.cpp
/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include "gazebo_hrh_gripper.hpp"
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>


namespace hsrb_gazebo_plugins {

namespace {

/// ハンドの最大開き幅（両指先の間隔）[mm]
/// 詳細はメカ部分の設計書参照
const double kMaxOpenWidth = 140.0 / 1000.0;

/// ハンドの最小開き幅[deg]
/// 詳細はメカ部分の設計書参照。この値は近似値。
const double kMinFingerDegrees = -7.0;

/// ハンドの最大開き幅[deg]
/// 詳細はメカ部分の設計書参照。この値は近似値。
const double kMaxFingerDegrees = 70.0;

/// ハンド各関節の最大出力トルク[Nm]
/// 直接サーボしている関節だけでなく、他の関節にもこの値が適用される。
/// [Nm] 設計的な根拠なし。これ以上に上げていくと物理演算が安定しない
const double kDefaultMaxTorque = 10.0;

/// ハンドエラーに対する感度[rad]
const double kDefaultSensitiveness = 0.01;

/// ハンドのゴール到達許容時間[s]
const double kDefaultGoalTimeTolerance = 5.0;

/// ハンドばね関節の最大出力トルク[Nm]
/// [Nm] 設計的な根拠なし。これ以上に上げていくと物理演算が安定しない
const double kDefaultMaxSpringJointTorque = 7.0;

/// 握りこみアクションのタイムアウト時間 [s]
const double kDefaultStallTimeout = 10.0;

/// 実機と同様のトルク性能を発揮させるためのゲイン値
const double kMaxTorqueGain = 1000.0;

/// 実機と同様の名前にするためのプレフィックス
const char* const kGripperNamePrefix = "gripper_controller/";
}  // anonymous namespace


GZ_REGISTER_MODEL_PLUGIN(GazeboHrhGripper);


inline double clamp(double value, double min, double max) {
  assert(min < max);
  if (value < min) {
    return min;
  }
  if (value > max) {
    return max;
  }
  return value;
}


/// ハンド開き幅[mm]から指の指令角度を計算する関数
inline double angle_from_position(double position) {
  const double x = clamp(position / kMaxOpenWidth + std::sin(angles::from_degrees(kMinFingerDegrees)),
                         -M_PI / 2.0, M_PI / 2.0);
  return std::asin(x);
}


GazeboHrhGripper::GazeboHrhGripper() {
    /// 本当は変数を色々初期化するべきだが、Loadメンバー関数があるので省略。
}


GazeboHrhGripper::~GazeboHrhGripper() {
  queue_.clear();
  queue_.disable();
  if (ros_node_) {
    ros_node_->shutdown();
  }
  if (spin_thread_) {
    spin_thread_->join();
  }

#if GAZEBO_MAJOR_VERSION >= 8
  update_connection_.reset();
#else
  gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
#endif
}


void GazeboHrhGripper::LoadParameters() {
  robot_namespace_ = "";
  if (sdf_->HasElement("robot_namespace")) {
    robot_namespace_ = sdf_->GetElement("robot_namespace")->Get<std::string>();
  } else {
    // ネームスペースが指定されない場合、Gazebo中のモデル名からつける。
    std::string name = model_->GetScopedName();
    std::vector<std::string> splitted;
    boost::split(splitted, name, boost::is_any_of("::"));
    robot_namespace_ = splitted.back();
  }

  if (sdf_->HasElement("update_rate")) {
    double update_rate = sdf_->GetElement("update_rate")->Get<double>();
    if (std::fabs(update_rate) < std::numeric_limits<double>::epsilon()) {
      update_rate = 60.0;
    }
    update_period_.fromSec(1.0 / update_rate);
  } else {
    update_period_.fromSec(1.0 / 60.0);
  }

  if (sdf_->HasElement("max_torque")) {
    max_torque_ = sdf_->GetElement("max_torque")->Get<double>();
  } else {
    max_torque_ = kDefaultMaxTorque;
  }

  if (sdf_->HasElement("max_spring_joint_torque")) {
    max_spring_joint_torque_ = sdf_->GetElement("max_spring_joint_torque")->Get<double>();
  } else {
    max_spring_joint_torque_ = kDefaultMaxSpringJointTorque;
  }

  if (sdf_->HasElement("stall_timeout")) {
    stall_timeout_.fromSec(sdf_->GetElement("stall_timeout")->Get<double>());
  } else {
    stall_timeout_.fromSec(kDefaultStallTimeout);
  }

  if (sdf_->HasElement("error_tolerance")) {
    error_tolerance_ = sdf_->GetElement("error_tolerance")->Get<double>();
  } else {
    error_tolerance_ = angles::from_degrees(1);
  }

  if (sdf_->HasElement("motor_joint_name")) {
    motor_joint_name_ = sdf_->GetElement("motor_joint_name")->Get<std::string>();
  } else {
    motor_joint_name_ = "hand_motor_joint";
  }

  if (sdf_->HasElement("sensitiveness")) {
      sensitiveness_ = sdf_->GetElement("sensitiveness")->Get<double>();
  } else {
      sensitiveness_ = kDefaultSensitiveness;
  }

  const boost::array<std::string, kNumJoints> base_names = {{
     "motor_joint",
     "l_proximal_joint",
     "l_spring_proximal_joint",
     "l_mimic_distal_joint",
     "l_distal_joint",
     "r_proximal_joint",
     "r_spring_proximal_joint",
     "r_mimic_distal_joint",
     "r_distal_joint",
  }};
  joint_names_.clear();
  BOOST_FOREACH(const std::string& joint_name, base_names) {
    std::string tag_name = joint_name + "_name";
    if (sdf_->HasElement(tag_name)) {
      joint_names_.push_back(sdf_->GetElement(tag_name)->Get<std::string>());
    } else {
      joint_names_.push_back("hand_" + joint_name);
    }
  }
}


void GazeboHrhGripper::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ = model;
  sdf_ = sdf;
  world_ = model->GetWorld();

  last_updated_time_ = ros::Time(0, 0);
  expected_update_time_ = ros::Time(0, 0);

  command_position_ = 0.0;
  command_max_torque_ = kDefaultMaxTorque;

  LoadParameters();

  ros_node_.reset(new ros::NodeHandle(robot_namespace_));

  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node in Gazebo server has not been initialized, failed to load plugin.");
    return;
  }

  gazebo_joints_.resize(joint_names_.size());
  pid_controllers_.resize(joint_names_.size());

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    gazebo::physics::JointPtr joint = model_->GetJoint(joint_names_[i]);
    if (!joint) {
      ROS_FATAL_STREAM("Joint(" << joint_names_[i] << ") was not found");
    }
    gazebo_joints_[i] = joint;
    gazebo_joints_[i]->SetEffortLimit(0, max_torque_ * kMaxTorqueGain);
    if (i == kLeftProximalJoint || i == kRightProximalJoint ||
        i == kLeftDistalJoint || i == kRightDistalJoint ||
        i == kRightSpringProximalJoint || i == kLeftSpringProximalJoint) {
      const ros::NodeHandle nh(*ros_node_, kGripperNamePrefix + std::string("pid_gains/") + joint_names_[i]);
      pid_controllers_[i].init(nh, true);
    }
  }

  spin_thread_.reset(new boost::thread(boost::bind(&GazeboHrhGripper::SpinThread, this)));

  follow_jnt_traj_action_server_.reset(
      new FollowJointTrajectoryActionServer(
          *ros_node_,
          kGripperNamePrefix + std::string("follow_joint_trajectory"),
          boost::bind(&GazeboHrhGripper::FollowJointTrajectoryActionGoalCB, this, _1),
          boost::bind(&GazeboHrhGripper::FollowJointTrajectoryActionCancelCB, this, _1),
          false));

  grasp_action_server_.reset(
      new GripperApplyEffortActionServer(
          *ros_node_,
          kGripperNamePrefix + std::string("grasp"),
          boost::bind(&GazeboHrhGripper::GraspActionGoalCB, this, _1),
          boost::bind(&GazeboHrhGripper::GraspActionCancelCB, this, _1),
          false));

  apply_force_action_server_.reset(
      new GripperApplyEffortActionServer(
          *ros_node_,
          kGripperNamePrefix + std::string("apply_force"),
          boost::bind(&GazeboHrhGripper::ApplyForceActionGoalCB, this, _1),
          boost::bind(&GazeboHrhGripper::ApplyForceActionCancelCB, this, _1),
          false));

  follow_jnt_traj_action_server_->start();
  grasp_action_server_->start();
  apply_force_action_server_->start();

  jnt_traj_command_sub_ = ros_node_->subscribe(kGripperNamePrefix + std::string("command"),
                                              1,
                                              &GazeboHrhGripper::JointTrajectoryCommandCB,
                                              this);

  // ゴール到達時刻の許容誤差取得
  // アクションでgoal_time_toleranceを指定された場合は上書きされる
  ros_node_->param(kGripperNamePrefix + std::string("position_goal_time_tolerance"),
                   default_goal_time_tolerance_,
                   kDefaultGoalTimeTolerance);
  if (default_goal_time_tolerance_ < 0.0) {
    ROS_WARN("position_goal_time_tolerance must be positive value"
             "or equal to zero. Using default 5.0[s]");
    default_goal_time_tolerance_ = kDefaultGoalTimeTolerance;
  }

  boost::function<void(const gazebo::common::UpdateInfo&)> fn = boost::bind(&GazeboHrhGripper::OnUpdate, this, _1);
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(fn);
}


void GazeboHrhGripper::CommandJoints(double command_position, double command_max_torque, ros::Duration dt) {
  const double max_torque = clamp(std::abs(command_max_torque), 0, max_torque_);
  ROS_DEBUG_STREAM_THROTTLE(1, "max_torque = " << max_torque << " clamped from " << command_max_torque);

  for (std::size_t i = 0; i < gazebo_joints_.size(); ++i) {
    gazebo_joints_[i]->SetEffortLimit(0, max_torque * kMaxTorqueGain);
  }

#if GAZEBO_MAJOR_VERSION >= 8
  const double ref_left = angles::normalize_angle(gazebo_joints_[kLeftProximalJoint]->Position(0));
  const double ref_right = angles::normalize_angle(gazebo_joints_[kRightProximalJoint]->Position(0));
#else
  const double ref_left = angles::normalize_angle(gazebo_joints_[kLeftProximalJoint]->GetAngle(0).Radian());
  const double ref_right = angles::normalize_angle(gazebo_joints_[kRightProximalJoint]->GetAngle(0).Radian());
#endif
  for (std::size_t i = 0; i < gazebo_joints_.size(); ++i) {
    switch (i) {
      case kLeftProximalJoint:
      case kRightProximalJoint: {
        const double target = angles::normalize_angle(command_position);
#if GAZEBO_MAJOR_VERSION >= 8
        const double state = angles::normalize_angle(gazebo_joints_[i]->Position(0));
#else
        const double state = angles::normalize_angle(gazebo_joints_[i]->GetAngle(0).Radian());
#endif
        const double error = target - state;
        if (std::fabs(error) >= sensitiveness_) {
            const double effort = clamp(pid_controllers_[i].computeCommand(error, dt), -max_torque_, max_torque_);
            gazebo_joints_[i]->SetForce(0, effort);
        }
        break;
      }
      case kLeftDistalJoint: {
        const double l_distal_joint_angle =
            -(ref_left +
#if GAZEBO_MAJOR_VERSION >= 8
              angles::normalize_angle(gazebo_joints_[kLeftSpringProximalJoint]->Position(0)));
        const double target = l_distal_joint_angle;
        const double state = angles::normalize_angle(gazebo_joints_[i]->Position(0));
#else
              angles::normalize_angle(gazebo_joints_[kLeftSpringProximalJoint]->GetAngle(0).Radian()));
        const double target = l_distal_joint_angle;
        const double state = angles::normalize_angle(gazebo_joints_[i]->GetAngle(0).Radian());
#endif
        const double error = target - state;
        if (std::fabs(error) >= sensitiveness_) {
          const double effort = clamp(pid_controllers_[i].computeCommand(error, dt), -max_torque_, max_torque_);
          gazebo_joints_[i]->SetForce(0, effort);
        }
        break;
      }
      case kRightDistalJoint: {
        const double r_distal_joint_angle =
            -(ref_right +
#if GAZEBO_MAJOR_VERSION >= 8
              angles::normalize_angle(gazebo_joints_[kRightSpringProximalJoint]->Position(0)));
        const double target = r_distal_joint_angle;
        const double state = angles::normalize_angle(gazebo_joints_[i]->Position(0));
#else
              angles::normalize_angle(gazebo_joints_[kRightSpringProximalJoint]->GetAngle(0).Radian()));
        const double target = r_distal_joint_angle;
        const double state = angles::normalize_angle(gazebo_joints_[i]->GetAngle(0).Radian());
#endif
        const double error = target - state;
        if (std::fabs(error) >= sensitiveness_) {
          const double effort = clamp(pid_controllers_[i].computeCommand(error, dt), -max_torque_, max_torque_);
          gazebo_joints_[i]->SetForce(0, effort);
        }
        break;
      }
      case kRightSpringProximalJoint:
      case kLeftSpringProximalJoint: {
        const double target = angles::normalize_angle(0.0);
#if GAZEBO_MAJOR_VERSION >= 8
        const double state = angles::normalize_angle(gazebo_joints_[i]->Position(0));
#else
        const double state = angles::normalize_angle(gazebo_joints_[i]->GetAngle(0).Radian());
#endif
        const double error = target - state;
        if (std::fabs(error) >= sensitiveness_) {
            const double effort = clamp(pid_controllers_[i].computeCommand(error, dt),
                                        -max_spring_joint_torque_,
                                        max_spring_joint_torque_);
            gazebo_joints_[i]->SetForce(0, effort);
        }
        break;
      }
      case kMotorJoint: {
        double target = ref_left;
#if GAZEBO_MAJOR_VERSION >= 4
        gazebo_joints_[i]->SetPosition(0, target);
#else
        gazebo_joints_[i]->SetAngle(0, target);
#endif
        break;
      }
      default: {
#if GAZEBO_MAJOR_VERSION >= 4
        gazebo_joints_[i]->SetPosition(0, 0);
#else
        gazebo_joints_[i]->SetAngle(0, 0);
#endif
        break;
      }
    }
  }
}


bool GazeboHrhGripper::CommandFinished() {
  for (std::size_t i = 0; i < gazebo_joints_.size(); ++i) {
    switch (i) {
      case kLeftProximalJoint:
      case kRightProximalJoint: {
#if GAZEBO_MAJOR_VERSION >= 8
        if (std::fabs(gazebo_joints_[i]->Position(0) - command_position_) > error_tolerance_) {
#else
        if (std::fabs(gazebo_joints_[i]->GetAngle(0).Radian() - command_position_) > error_tolerance_) {
#endif
          return false;
        }
        break;
      }
    }
  }
  return true;
}


void GazeboHrhGripper::OnUpdate(const gazebo::common::UpdateInfo& info) {
  ros::Time now(info.simTime.sec, info.simTime.nsec);

  if (last_updated_time_.is_zero()) {
    last_updated_time_ = now;
    expected_update_time_ = now + update_period_;
    return;
  }

  if (now >= expected_update_time_) {
    ros::Duration dt = now - last_updated_time_;
    command_position_ = clamp(command_position_,
                              angles::from_degrees(kMinFingerDegrees),
                              angles::from_degrees(kMaxFingerDegrees));
    ROS_DEBUG_STREAM_THROTTLE(1, "command_position_ = "
                              << command_position_
                              << " max_torque = "
                              << command_max_torque_);
    ROS_DEBUG_STREAM_THROTTLE(1, "FOLLOW_JOINT_TRAJECTORY "
                              << static_cast<int>(follow_jnt_traj_action_active_goal_.isValid()));
    CommandJoints(command_position_, command_max_torque_, dt);

    if (!action_succeeded_signal_.empty()) {
      if (CommandFinished()) {
        action_succeeded_signal_();
        action_succeeded_signal_.disconnect_all_slots();
      }
    }
    if (!action_timedout_signal_.empty()) {
      if (now > expected_goal_time_) {
        action_timedout_signal_();
        action_timedout_signal_.disconnect_all_slots();
      }
    }

    expected_update_time_ = now + update_period_;
    last_updated_time_ = now;
  }
}


void GazeboHrhGripper::SpinThread() {
  while (ros_node_->ok()) {
    queue_.callAvailable(ros::WallDuration(0.01));
  }
}


void GazeboHrhGripper::CancelAllGoals() {
  // 新しいゴールが来たら上書き
  if (follow_jnt_traj_action_active_goal_.isValid() &&
      follow_jnt_traj_action_active_goal_.getGoalStatus().status ==
      actionlib_msgs::GoalStatus::ACTIVE) {
    follow_jnt_traj_action_active_goal_.setCanceled();
    follow_jnt_traj_action_active_goal_ = FollowJointTrajectoryActionGoalHandle();
    action_succeeded_signal_.disconnect_all_slots();
    action_timedout_signal_.disconnect_all_slots();
  }
  if (grasp_action_active_goal_.isValid() &&
      grasp_action_active_goal_.getGoalStatus().status ==
      actionlib_msgs::GoalStatus::ACTIVE) {
      grasp_action_active_goal_.setCanceled();
    grasp_action_active_goal_ = GripperApplyEffortActionGoalHandle();
    action_succeeded_signal_.disconnect_all_slots();
    action_timedout_signal_.disconnect_all_slots();
  }
  if (apply_force_action_active_goal_.isValid() &&
      apply_force_action_active_goal_.getGoalStatus().status ==
      actionlib_msgs::GoalStatus::ACTIVE) {
    apply_force_action_active_goal_.setCanceled();
    apply_force_action_active_goal_ = GripperApplyEffortActionGoalHandle();
    action_succeeded_signal_.disconnect_all_slots();
    action_timedout_signal_.disconnect_all_slots();
  }
}

void GazeboHrhGripper::FollowJointTrajectoryActionGoalCB(FollowJointTrajectoryActionGoalHandle goal) {
  // 新しいゴールが来たら上書き
  CancelAllGoals();
  goal.setAccepted();
  const trajectory_msgs::JointTrajectory& traj = goal.getGoal()->trajectory;
  std::vector<std::string>::const_iterator motor_joint_iter = std::find(traj.joint_names.begin(),
                                                                        traj.joint_names.end(),
                                                                        motor_joint_name_);
  if (motor_joint_iter == traj.joint_names.end()) {
    // ジョイント名が見つからない時は成功として扱う。
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    goal.setSucceeded(result);
    return;
  }
  const trajectory_msgs::JointTrajectoryPoint& last_point = traj.points.back();
  std::vector<double>::const_iterator position_iter = last_point.positions.begin();
  std::advance(position_iter, std::distance(traj.joint_names.begin(), motor_joint_iter));

  command_position_ = *position_iter;

  ros::Duration goal_time_tolerance;
  if (goal.getGoal()->goal_time_tolerance == ros::Duration(0.0)) {
    goal_time_tolerance =
      ros::Duration(default_goal_time_tolerance_);
  } else if (goal.getGoal()->goal_time_tolerance < ros::Duration(0.0)) {
    goal_time_tolerance = ros::Duration(0.0);
  } else {
    goal_time_tolerance = goal.getGoal()->goal_time_tolerance;
  }

  expected_goal_time_ = goal.getGoalID().stamp + last_point.time_from_start + goal_time_tolerance;
  follow_jnt_traj_action_active_goal_ = goal;
  action_succeeded_signal_.connect(boost::bind(&GazeboHrhGripper::FollowJointTrajectoryActionSucceededCB, this));
  action_timedout_signal_.connect(boost::bind(&GazeboHrhGripper::FollowJointTrajectoryActionTimedoutCB, this));
}


void GazeboHrhGripper::FollowJointTrajectoryActionCancelCB(FollowJointTrajectoryActionGoalHandle goal) {
  if (follow_jnt_traj_action_active_goal_.isValid() && follow_jnt_traj_action_active_goal_ == goal) {
    follow_jnt_traj_action_active_goal_.setCanceled();
    follow_jnt_traj_action_active_goal_ = FollowJointTrajectoryActionGoalHandle();
    last_updated_time_ = ros::Time(0, 0);
  }
}


void GazeboHrhGripper::FollowJointTrajectoryActionSucceededCB() {
  if (follow_jnt_traj_action_active_goal_.isValid()) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    follow_jnt_traj_action_active_goal_.setSucceeded(result);
    follow_jnt_traj_action_active_goal_ = FollowJointTrajectoryActionGoalHandle();
  }
}


void GazeboHrhGripper::FollowJointTrajectoryActionTimedoutCB() {
  if (follow_jnt_traj_action_active_goal_.isValid()) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
    follow_jnt_traj_action_active_goal_.setSucceeded(result);
    follow_jnt_traj_action_active_goal_ = FollowJointTrajectoryActionGoalHandle();
  }
}


void GazeboHrhGripper::GraspActionGoalCB(GripperApplyEffortActionGoalHandle goal) {
  // 新しいゴールが来たら上書き
  CancelAllGoals();
  goal.setAccepted();
  double effort = goal.getGoal()->effort;
  if (effort == 0.0) {
#if GAZEBO_MAJOR_VERSION >= 8
    command_position_ = gazebo_joints_[kLeftProximalJoint]->Position(0);
#else
    command_position_ = gazebo_joints_[kLeftProximalJoint]->GetAngle(0).Radian();
#endif
    command_max_torque_ = kDefaultMaxTorque;
  } else if (effort < 0.0) {
    command_position_ = angles::from_degrees(kMinFingerDegrees);
    command_max_torque_ = effort;
  } else {
    command_position_ = angles::from_degrees(kMaxFingerDegrees);
    command_max_torque_ = effort;
  }

  expected_goal_time_ = goal.getGoalID().stamp + stall_timeout_;
  grasp_action_active_goal_ = goal;
  action_succeeded_signal_.connect(boost::bind(&GazeboHrhGripper::GraspActionSucceededCB, this));
  action_timedout_signal_.connect(boost::bind(&GazeboHrhGripper::GraspActionTimedoutCB, this));
}


void GazeboHrhGripper::GraspActionCancelCB(GripperApplyEffortActionGoalHandle goal) {
  if (grasp_action_active_goal_.isValid() && grasp_action_active_goal_ == goal) {
    grasp_action_active_goal_.setCanceled();
    grasp_action_active_goal_ = GripperApplyEffortActionGoalHandle();
    last_updated_time_ = ros::Time(0, 0);
  }
}


void GazeboHrhGripper::GraspActionSucceededCB() {
  if (grasp_action_active_goal_.isValid()) {
    tmc_control_msgs::GripperApplyEffortResult result;
    result.stalled = true;
    result.effort = command_max_torque_;
    grasp_action_active_goal_.setSucceeded(result);
    grasp_action_active_goal_ = GripperApplyEffortActionGoalHandle();
  }
}


void GazeboHrhGripper::GraspActionTimedoutCB() {
  if (grasp_action_active_goal_.isValid()) {
    tmc_control_msgs::GripperApplyEffortResult result;
    result.stalled = false;
    result.effort = command_max_torque_;
    grasp_action_active_goal_.setSucceeded(result);
    grasp_action_active_goal_ = GripperApplyEffortActionGoalHandle();
  }
}


void GazeboHrhGripper::ApplyForceActionGoalCB(GripperApplyEffortActionGoalHandle goal) {
  // 新しいゴールが来たら上書き
  CancelAllGoals();
  goal.setAccepted();

  command_position_ = 0.0;
  command_max_torque_ = goal.getGoal()->effort;
  expected_goal_time_ = goal.getGoalID().stamp + stall_timeout_;
  apply_force_action_active_goal_ = goal;
  action_succeeded_signal_.connect(boost::bind(&GazeboHrhGripper::ApplyForceActionSucceededCB, this));
  action_timedout_signal_.connect(boost::bind(&GazeboHrhGripper::ApplyForceActionTimedoutCB, this));
}


void GazeboHrhGripper::ApplyForceActionCancelCB(GripperApplyEffortActionGoalHandle goal) {
  if (apply_force_action_active_goal_.isValid() && apply_force_action_active_goal_ == goal) {
    apply_force_action_active_goal_.setCanceled();
    apply_force_action_active_goal_ = GripperApplyEffortActionGoalHandle();
    last_updated_time_ = ros::Time(0, 0);
  }
}


void GazeboHrhGripper::ApplyForceActionSucceededCB() {
  if (apply_force_action_active_goal_.isValid()) {
    tmc_control_msgs::GripperApplyEffortResult result;
    result.stalled = true;
    result.effort = command_max_torque_;
    apply_force_action_active_goal_.setSucceeded(result);
    apply_force_action_active_goal_ = GripperApplyEffortActionGoalHandle();
  }
}


void GazeboHrhGripper::ApplyForceActionTimedoutCB() {
  if (apply_force_action_active_goal_.isValid()) {
    tmc_control_msgs::GripperApplyEffortResult result;
    result.stalled = false;
    result.effort = command_max_torque_;
    apply_force_action_active_goal_.setSucceeded(result);
    apply_force_action_active_goal_ = GripperApplyEffortActionGoalHandle();
  }
}


void GazeboHrhGripper::JointTrajectoryCommandCB(trajectory_msgs::JointTrajectory::ConstPtr msg) {
  std::vector<std::string>::const_iterator motor_joint_iter = std::find(msg->joint_names.begin(),
                                                                        msg->joint_names.end(),
                                                                        motor_joint_name_);
  if (motor_joint_iter == msg->joint_names.end()) {
    // ジョイント名が見つからない
    ROS_ERROR("Failed: invalid joint names.");
    return;
  }
  const trajectory_msgs::JointTrajectoryPoint& last_point = msg->points.back();
  std::vector<double>::const_iterator position_iter = last_point.positions.begin();
  std::advance(position_iter, std::distance(msg->joint_names.begin(), motor_joint_iter));

  command_position_ = *position_iter;
  command_max_torque_ = 0.020;
}

}  // namespace hsrb_gazebo_plugins
