/// @brief HSR-B用ハンド制御プラグイン
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
#ifndef HSRB_GAZEBO_PLUGINS_HRH_GRIPPER_PLUGIN_HPP_
#define HSRB_GAZEBO_PLUGINS_HRH_GRIPPER_PLUGIN_HPP_

#include <string>
#include <vector>

#include <actionlib/server/action_server.h>

#include <boost/scoped_ptr.hpp>
#include <boost/signals2.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_toolbox/pid.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>


namespace hsrb_gazebo_plugins {

///
class GazeboHrhGripper : public gazebo::ModelPlugin {
 public:
  GazeboHrhGripper();

  virtual ~GazeboHrhGripper();

  virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

 private:
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionServer;
  typedef boost::shared_ptr<FollowJointTrajectoryActionServer> FollowJointTrajectoryActionServerPtr;
  typedef FollowJointTrajectoryActionServer::GoalHandle FollowJointTrajectoryActionGoalHandle;

  typedef actionlib::ActionServer<tmc_control_msgs::GripperApplyEffortAction> GripperApplyEffortActionServer;
  typedef boost::shared_ptr<GripperApplyEffortActionServer> GripperApplyEffortActionServerPtr;
  typedef GripperApplyEffortActionServer::GoalHandle GripperApplyEffortActionGoalHandle;

  enum {
    kMotorJoint,
    kLeftProximalJoint,
    kLeftSpringProximalJoint,
    kLeftMimicDistalJoint,
    kLeftDistalJoint,
    kRightProximalJoint,
    kRightSpringProximalJoint,
    kRightMimicDistalJoint,
    kRightDistalJoint,
    kNumJoints
  };

  void LoadParameters();

  void OnUpdate(const gazebo::common::UpdateInfo& info);
  void SpinThread();

  void CommandJoints(double command_angle, double max_effort, ros::Duration dt);
  bool CommandFinished();

  void CancelAllGoals();

  void FollowJointTrajectoryActionGoalCB(FollowJointTrajectoryActionGoalHandle goal);
  void FollowJointTrajectoryActionCancelCB(FollowJointTrajectoryActionGoalHandle goal);
  void FollowJointTrajectoryActionSucceededCB();
  void FollowJointTrajectoryActionTimedoutCB();

  void GraspActionGoalCB(GripperApplyEffortActionGoalHandle goal);
  void GraspActionCancelCB(GripperApplyEffortActionGoalHandle goal);
  void GraspActionSucceededCB();
  void GraspActionTimedoutCB();

  void ApplyForceActionGoalCB(GripperApplyEffortActionGoalHandle goal);
  void ApplyForceActionCancelCB(GripperApplyEffortActionGoalHandle goal);
  void ApplyForceActionSucceededCB();
  void ApplyForceActionTimedoutCB();

  void JointTrajectoryCommandCB(trajectory_msgs::JointTrajectory::ConstPtr msg);

  gazebo::physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_connection_;

  std::vector<std::string> joint_names_;
  std::vector<gazebo::physics::JointPtr> gazebo_joints_;
  std::vector<control_toolbox::Pid> pid_controllers_;

  ros::CallbackQueue queue_;
  boost::scoped_ptr<boost::thread> spin_thread_;

  double command_position_;
  double command_max_torque_;

  FollowJointTrajectoryActionServerPtr follow_jnt_traj_action_server_;
  FollowJointTrajectoryActionGoalHandle follow_jnt_traj_action_active_goal_;

  GripperApplyEffortActionServerPtr grasp_action_server_;
  GripperApplyEffortActionGoalHandle grasp_action_active_goal_;

  GripperApplyEffortActionServerPtr apply_force_action_server_;
  GripperApplyEffortActionGoalHandle apply_force_action_active_goal_;

  ros::Subscriber jnt_traj_command_sub_;
  boost::scoped_ptr<ros::NodeHandle> ros_node_;

  ros::Duration update_period_;
  ros::Time expected_update_time_;
  ros::Time last_updated_time_;
  ros::Time expected_goal_time_;
  ros::Duration stall_timeout_;
  double max_torque_;
  double max_spring_joint_torque_;
  double error_tolerance_;
  double default_goal_time_tolerance_;
  std::string robot_namespace_;
  std::string motor_joint_name_;

  double p_gain_;
  double i_gain_;
  double d_gain_;
  double i_clamp_;

  double max_open_width_;
  double min_finger_degrees_;
  double max_finger_degrees_;
  double sensitiveness_;

  boost::signals2::signal<void()> action_succeeded_signal_;
  boost::signals2::signal<void()> action_timedout_signal_;
};

}  // namespace hsrb_gazebo_plugins

#endif  // HSRB_GAZEBO_PLUGINS_HRH_GRIPPER_PLUGIN_HPP_
