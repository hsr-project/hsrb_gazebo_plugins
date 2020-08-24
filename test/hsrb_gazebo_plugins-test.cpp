/*
Copyright (c) 2015 TOYOTA MOTOR CORPORATION
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
#include <cmath>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_suction/SuctionControlAction.h>

namespace {
/// 握り込みアクション実行の許容時間[s]
const double kTimeout = 10.0;
/// 目標点に対し許容される位置誤差 [m]
const double kPositionErrorThreshold = 0.01;
/// distal関節の角度オフセット[rad]
const double kDistalJointAngleOffset = 0.087;
/// ハンドの最大開き幅[deg]
const double kMaxFingerDegrees = 70.0;
/// ハンドの最小開き幅[deg]
const double kMinFingerDegrees = -3.0;
/// ばね関節の最大開き幅[deg]
const double kSpringProximalJointAngleMax = 0.698;
/// ばね関節の最小開き幅[deg]
const double kSpringProximalJointAngleMin = 0.0;
/// 制御可能な出力トルク[Nm]
const double kSuccessEffort = 0.006;
/// 制御不可能な出力トルク[Nm]
const double kFailureEffort = 0.005;
/// 目標到達時間に対し許容される誤差[s]
const double kGoalTimeTolerance = 5.0;
/// 目標点の予測到達時間[s]
const double kTimeFromStart = 5.0;
/// 動作後、ハンドや物体の状態が安定するまでの時間[s]
const double kStabilizationTime = 5.0;
/// 物体把持チェックの際に閾値となる距離 [m]
const double kGraspingPositionThreshold = 0.15;
}  // unnamed namespace

namespace hsrb_gazebo_plugins {

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

enum HandOperation {
  kOpenHand,
  kCloseHand
};

// ハンド各関節軸の状態チェック(間に物体が無い場合)
void ExpectHandJointStateWithoutObject(sensor_msgs::JointState joint_state,
                                       double goal_position) {
  // hand_motor_jointは目標値まで到達する
  EXPECT_NEAR(joint_state.position[kMotorJoint],
              goal_position,
              kPositionErrorThreshold);
  // 左指
  EXPECT_EQ(joint_state.position[kLeftProximalJoint],
            joint_state.position[kMotorJoint]);
  // sprint_jointは伸びない
  EXPECT_NEAR(joint_state.position[kLeftSpringProximalJoint],
              kSpringProximalJointAngleMin,
              kPositionErrorThreshold);
  EXPECT_EQ(joint_state.position[kLeftMimicDistalJoint],
            -joint_state.position[kLeftSpringProximalJoint]);
  EXPECT_NEAR(joint_state.position[kLeftDistalJoint],
              -(joint_state.position[kLeftProximalJoint] +
                kDistalJointAngleOffset),
              kPositionErrorThreshold);
  // 右指
  EXPECT_EQ(joint_state.position[kRightProximalJoint],
            joint_state.position[kMotorJoint]);
  // sprint_jointは伸びない
  EXPECT_NEAR(joint_state.position[kRightSpringProximalJoint],
              kSpringProximalJointAngleMin,
              kPositionErrorThreshold);
  EXPECT_EQ(joint_state.position[kRightMimicDistalJoint],
            -joint_state.position[kRightSpringProximalJoint]);
  EXPECT_NEAR(joint_state.position[kRightDistalJoint],
              -(joint_state.position[kRightProximalJoint] +
                kDistalJointAngleOffset),
              kPositionErrorThreshold);
}

// ハンド各関節軸の状態チェック(間に物体がある場合)
void ExpectHandJointStateWithObject(sensor_msgs::JointState joint_state) {
  // hand_motor_jointの値はわからない
  // 左指
  EXPECT_EQ(joint_state.position[kLeftProximalJoint],
            joint_state.position[kMotorJoint]);
  // sprint_jointは最大まで伸びる
  EXPECT_NEAR(joint_state.position[kLeftSpringProximalJoint],
              kSpringProximalJointAngleMax,
              kPositionErrorThreshold);
  EXPECT_EQ(joint_state.position[kLeftMimicDistalJoint],
            -joint_state.position[kLeftSpringProximalJoint]);
  EXPECT_NEAR(joint_state.position[kLeftDistalJoint],
              -(joint_state.position[kLeftProximalJoint] +
                kDistalJointAngleOffset),
              kPositionErrorThreshold);
  // 右指
  EXPECT_EQ(joint_state.position[kRightProximalJoint],
            joint_state.position[kLeftProximalJoint]);
  // sprint_jointは最大まで伸びる
  EXPECT_NEAR(joint_state.position[kRightSpringProximalJoint],
              kSpringProximalJointAngleMax,
              kPositionErrorThreshold);
  EXPECT_EQ(joint_state.position[kRightMimicDistalJoint],
            -joint_state.position[kRightSpringProximalJoint]);
  EXPECT_NEAR(joint_state.position[kRightDistalJoint],
              -(joint_state.position[kRightProximalJoint] +
                kDistalJointAngleOffset),
              kPositionErrorThreshold);
}

// 物体の把持状態をチェック
void ExpectGraspState(bool is_grasping) {
  gazebo_msgs::GetLinkState get_link_state_srv;
  get_link_state_srv.request.link_name = "small_bottle::link";
  get_link_state_srv.request.reference_frame = "wrist_roll_link";
  ros::service::call("/gazebo/get_link_state", get_link_state_srv);
  if (is_grasping) {
    EXPECT_NEAR(get_link_state_srv.response.link_state.pose.position.x,
                0.0, kGraspingPositionThreshold);
  } else {
    EXPECT_GT(get_link_state_srv.response.link_state.pose.position.x,
              kGraspingPositionThreshold);
  }
}


/// トピックが発行されたかテストするクラス
template <class MessageType>
class TestSubscriber {
 public:
  TestSubscriber() : is_sub_(false) {}
  void Init(const std::string& topic) {
    ros::NodeHandle node;
    subscriber_ = node.subscribe(
        topic, 1,
        &TestSubscriber<MessageType>::Callback, this);
  }

  bool WaitForMessage(const ros::Duration& timeout,
                      MessageType& dst_value) {
    is_sub_ = false;
    ros::Time start = ros::Time::now();
    ros::Rate rate(10.0);
    while ((is_sub_ == false) &&
           ((ros::Time::now() - start) < timeout)) {
      ros::spinOnce();
      rate.sleep();
    }
    if (is_sub_ == false) {
      return false;
    }
    dst_value = value_;
    return true;
  }

  bool WaitUntilConnectionEstablished(const ros::Duration& timeout) {
    ros::Time start = ros::Time::now();
    ros::Rate rate(10.0);
    while (subscriber_.getNumPublishers() == 0 &&
           (ros::Time::now() - start) < timeout) {
      ros::spinOnce();
      rate.sleep();
    }
    if (subscriber_.getNumPublishers() == 0) {
      return false;
    }
    return true;
  }

 private:
  void Callback(const MessageType& msg) {
    value_ = msg;
    is_sub_ = true;
  }

  ros::Subscriber subscriber_;
  MessageType value_;
  bool is_sub_;
};

class HsrbGazeboPluginsTest
    : public ::testing::Test {
 protected:
  HsrbGazeboPluginsTest() {
    ros::NodeHandle node;
    joint_trajectory_pub_ =
        node.advertise<trajectory_msgs::JointTrajectory>(
            "/hsrb/gripper_controller/command",
            1, false);
    command_suction_pub_ =
        node.advertise<std_msgs::Bool>(
            "/hsrb/command_suction",
            1, false);
    joint_state_sub.Init("/hsrb/robot_state/joint_states");
    suction_state_sub.Init("/hsrb/pressure_sensor");
  }

  virtual ~HsrbGazeboPluginsTest() {}

  virtual void SetUp() {
    ASSERT_TRUE(WaitUntilConnectionEstablished(kTimeout));
    ASSERT_TRUE(joint_state_sub.WaitUntilConnectionEstablished(
        ros::Duration(kTimeout)));
    ASSERT_TRUE(suction_state_sub.WaitUntilConnectionEstablished(
        ros::Duration(kTimeout)));
  }

  void SetHandState(HandOperation operation) {
    trajectory_msgs::JointTrajectory joint_trajectory_msg;
    joint_trajectory_msg.header.stamp = ros::Time::now();
    joint_trajectory_msg.joint_names.resize(1);
    joint_trajectory_msg.joint_names[0] = "hand_motor_joint";
    joint_trajectory_msg.points.resize(1);
    joint_trajectory_msg.points[0].positions.resize(1);
    joint_trajectory_msg.points[0].velocities.resize(1);
    joint_trajectory_msg.points[0].accelerations.resize(1);
    joint_trajectory_msg.points[0].effort.resize(1);
    joint_trajectory_msg.points[0].time_from_start = ros::Duration(kTimeFromStart);
    switch (operation) {
      case kOpenHand:
        joint_trajectory_msg.points[0].positions[0] = kMaxFingerDegrees * M_PI / 180;
        break;
      case kCloseHand:
        joint_trajectory_msg.points[0].positions[0] = 0.0;
        break;
      default:
        ROS_FATAL("SetHandState function needs arg 0 or 1. Now %d.", operation);
        return;
    }
    Publish(joint_trajectory_msg);
    ros::Duration(kTimeFromStart + kGoalTimeTolerance).sleep();
  }

  void SetModelPose(const std::string& model_name,
                    double position_x, double position_y, double position_z,
                    double orientation_x, double orientation_y, double orientation_z,
                    double orientation_w) {
    set_model_state_srv_.request.model_state.model_name = model_name;
    set_model_state_srv_.request.model_state.pose.position.x = position_x;
    set_model_state_srv_.request.model_state.pose.position.y = position_y;
    set_model_state_srv_.request.model_state.pose.position.z = position_z;
    set_model_state_srv_.request.model_state.pose.orientation.x = orientation_x;
    set_model_state_srv_.request.model_state.pose.orientation.y = orientation_y;
    set_model_state_srv_.request.model_state.pose.orientation.z = orientation_z;
    set_model_state_srv_.request.model_state.pose.orientation.w = orientation_w;
    set_model_state_srv_.request.model_state.reference_frame = "map";
    ros::service::call("/gazebo/set_model_state", set_model_state_srv_);
  }

  void SetHandToGraspCheckPos() {
    SetModelPose("hsrb", -0.40, 0.0, 0.20,
                 0.0, 0.707, 0.0, 0.707);
    ros::Duration(kStabilizationTime).sleep();
  }

  void SetSuctionState(bool suction_on) {
    std_msgs::Bool suction_msg;
    suction_msg.data = suction_on;
    command_suction_pub_.publish(suction_msg);
    ros::Duration(kStabilizationTime).sleep();
  }

  void ExpectSuctionState(bool is_suction_on) {
    std_msgs::Bool suction_state;
    EXPECT_TRUE(suction_state_sub.WaitForMessage(ros::Duration(5.0),
                                                 suction_state));
    EXPECT_EQ(suction_state.data, is_suction_on);
  }

  void SuctionAction() {
    SetModelPose("small_bottle", -1.0, 0.03, 0.03,
                 0.0, 0.707, 0.0, 0.707);
    SetModelPose("hsrb", -1.0, 0.0, 0.20,
                 0.0, 1.0, 0.0, 0.0);

    trajectory_msgs::JointTrajectory joint_trajectory_msg;
    joint_trajectory_msg.header.stamp = ros::Time::now();
    joint_trajectory_msg.joint_names.resize(1);
    joint_trajectory_msg.joint_names[0] = "hand_motor_joint";
    joint_trajectory_msg.points.resize(1);
    joint_trajectory_msg.points[0].positions.resize(1);
    joint_trajectory_msg.points[0].velocities.resize(1);
    joint_trajectory_msg.points[0].accelerations.resize(1);
    joint_trajectory_msg.points[0].effort.resize(1);
    joint_trajectory_msg.points[0].time_from_start = ros::Duration(kTimeFromStart);
    // 吸引OFFの際はハンドから物が確実に落ちるくらいの角度
    joint_trajectory_msg.points[0].positions[0] = 0.3;
    Publish(joint_trajectory_msg);
    ros::Duration(kTimeFromStart + kTimeout).sleep();

    SetHandToGraspCheckPos();
  }

  bool WaitUntilConnectionEstablished(double max_duration) {
    ros::Rate rate(10.0);
    ros::Time start_time = ros::Time::now();
    while ((joint_trajectory_pub_.getNumSubscribers() == 0) &&
           (ros::Time::now() - start_time) < ros::Duration(max_duration)) {
      rate.sleep();
    }
    return ((ros::Time::now() - start_time) < ros::Duration(max_duration));
  }


  void Publish(const trajectory_msgs::JointTrajectory& msg) {
    joint_trajectory_pub_.publish(msg);
  }

  TestSubscriber<sensor_msgs::JointState> joint_state_sub;
  TestSubscriber<std_msgs::Bool> suction_state_sub;

 private:
  ros::Publisher joint_trajectory_pub_;
  ros::Publisher command_suction_pub_;
  gazebo_msgs::SetModelState set_model_state_srv_;
};

// gripperのjoint_trajectoryの正常動作テスト
TEST_F(HsrbGazeboPluginsTest, GripperJointTrajectorySuccess) {
  sensor_msgs::JointState joint_state;
  trajectory_msgs::JointTrajectory joint_trajectory_msg;
  joint_trajectory_msg.header.stamp = ros::Time::now();
  joint_trajectory_msg.joint_names.resize(1);
  joint_trajectory_msg.joint_names[0] = "hand_motor_joint";
  joint_trajectory_msg.points.resize(1);
  joint_trajectory_msg.points[0].positions.resize(1);
  joint_trajectory_msg.points[0].velocities.resize(1);
  joint_trajectory_msg.points[0].accelerations.resize(1);
  joint_trajectory_msg.points[0].effort.resize(1);
  joint_trajectory_msg.points[0].time_from_start = ros::Duration(kTimeFromStart);

  // 最小角より小さいときは最小角になる
  joint_trajectory_msg.points[0].positions[0] = (kMinFingerDegrees - 10.0) * M_PI / 180;
  Publish(joint_trajectory_msg);
  ros::Duration(kTimeFromStart).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithoutObject(joint_state, kMinFingerDegrees * M_PI / 180);

  // 最小角と最大角の間のときはその角度になる
  joint_trajectory_msg.points[0].positions[0] = (kMaxFingerDegrees - kMinFingerDegrees) / 2.0 * M_PI / 180;
  Publish(joint_trajectory_msg);
  ros::Duration(kTimeFromStart).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithoutObject(joint_state, (kMaxFingerDegrees - kMinFingerDegrees) / 2.0 * M_PI / 180);

  // 最大角より大きいときは最大角になる
  joint_trajectory_msg.points[0].positions[0] = (kMaxFingerDegrees + 10.0) * M_PI / 180;
  Publish(joint_trajectory_msg);
  ros::Duration(kTimeFromStart).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithoutObject(joint_state, kMaxFingerDegrees * M_PI / 180);
}

// gripperのjoint_trajectoryの異常動作テスト
TEST_F(HsrbGazeboPluginsTest, GripperJointTrajectoryFailure) {
  SetHandState(kOpenHand);

  sensor_msgs::JointState joint_state;
  trajectory_msgs::JointTrajectory joint_trajectory_msg;
  joint_trajectory_msg.header.stamp = ros::Time::now();
  joint_trajectory_msg.joint_names.resize(1);
  joint_trajectory_msg.points.resize(1);
  joint_trajectory_msg.points[0].positions.resize(1);
  joint_trajectory_msg.points[0].velocities.resize(1);
  joint_trajectory_msg.points[0].accelerations.resize(1);
  joint_trajectory_msg.points[0].effort.resize(1);

  // 関節角度名が存在しないとき
  joint_trajectory_msg.joint_names[0] = "hand_motro_joint";
  joint_trajectory_msg.points[0].positions[0] = 1.0;
  Publish(joint_trajectory_msg);
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithoutObject(joint_state, kMaxFingerDegrees * M_PI / 180);
}

// gripperのGraspアクション正常動作テスト
TEST_F(HsrbGazeboPluginsTest, GripperGraspActionSuccess) {
  SetHandState(kOpenHand);

  sensor_msgs::JointState joint_state;
  actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction>
      action_client("/hsrb/gripper_controller/grasp", true);
  ASSERT_TRUE(action_client.waitForServer(ros::Duration(kTimeout)));
  tmc_control_msgs::GripperApplyEffortGoal goal;

  // アクションが成功するeffort値を入れたとき
  goal.effort = kSuccessEffort;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeout)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(true,
            action_client.getResult()->stalled);

  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                                joint_state));
  ExpectHandJointStateWithoutObject(joint_state, 0.0);

  // 本来は負値を入れる
  SetHandState(kOpenHand);
  goal.effort = -kSuccessEffort;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeout)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(true,
            action_client.getResult()->stalled);

  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                                joint_state));
  ExpectHandJointStateWithoutObject(joint_state, 0.0);
}

// gripperのGraspアクション異常動作テスト
TEST_F(HsrbGazeboPluginsTest, GripperGraspActionFailure) {
  SetHandState(kOpenHand);

  sensor_msgs::JointState joint_state;
  actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction>
      action_client("/hsrb/gripper_controller/grasp", true);
  ASSERT_TRUE(action_client.waitForServer(ros::Duration(kTimeout)));
  tmc_control_msgs::GripperApplyEffortGoal goal;

  // effortが足りずタイムアウト
  goal.effort = kFailureEffort;
  action_client.sendGoal(goal);
  ASSERT_FALSE(action_client.waitForResult(ros::Duration(kTimeout)));
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(1.0)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(false,
            action_client.getResult()->stalled);
}

// gripperのApplyForceアクション正常動作テスト
TEST_F(HsrbGazeboPluginsTest, GripperApplyForceActionSuccess) {
  SetHandState(kOpenHand);

  sensor_msgs::JointState joint_state;
  actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction>
      action_client("/hsrb/gripper_controller/apply_force", true);
  ASSERT_TRUE(action_client.waitForServer(ros::Duration(kTimeout)));
  tmc_control_msgs::GripperApplyEffortGoal goal;

  // アクションが成功するeffort値を入れたとき
  goal.effort = kSuccessEffort;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeout)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(true,
            action_client.getResult()->stalled);

  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                                joint_state));
  ExpectHandJointStateWithoutObject(joint_state, 0.0);

  // 本来は負値を入れる
  SetHandState(kOpenHand);
  goal.effort = -kSuccessEffort;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeout)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(true,
            action_client.getResult()->stalled);

  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                                joint_state));
  ExpectHandJointStateWithoutObject(joint_state, 0.0);
}

// gripperのApplyEffortアクション異常動作テスト
TEST_F(HsrbGazeboPluginsTest, GripperApplyForceActionFailure) {
  SetHandState(kOpenHand);

  sensor_msgs::JointState joint_state;
  actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction>
      action_client("/hsrb/gripper_controller/apply_force", true);
  ASSERT_TRUE(action_client.waitForServer(ros::Duration(kTimeout)));
  tmc_control_msgs::GripperApplyEffortGoal goal;

  // effortが足りずタイムアウト
  goal.effort = kFailureEffort;
  action_client.sendGoal(goal);
  ASSERT_FALSE(action_client.waitForResult(ros::Duration(kTimeout)));
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(1.0)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(false,
            action_client.getResult()->stalled);
}

// gripperのFollowJointTrajectoryアクション正常動作テスト
TEST_F(HsrbGazeboPluginsTest, GripperFollowJointTrajectoryActionSuccess) {
  SetHandState(kOpenHand);

  sensor_msgs::JointState joint_state;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      action_client("/hsrb/gripper_controller/follow_joint_trajectory", true);
  ASSERT_TRUE(action_client.waitForServer(ros::Duration(kTimeout)));
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.goal_time_tolerance = ros::Duration(kGoalTimeTolerance);
  goal.trajectory.joint_names.resize(1);
  goal.trajectory.joint_names[0] = "hand_motor_joint";
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(1);
  goal.trajectory.points[0].time_from_start = ros::Duration(kTimeFromStart);

  // 最小角より小さいときは最小角になる
  goal.trajectory.points[0].positions[0] = (kMinFingerDegrees - 10.0) * M_PI / 180;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeFromStart
                                                        + kGoalTimeTolerance)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
            action_client.getResult()->error_code);
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithoutObject(joint_state, kMinFingerDegrees * M_PI / 180);

  // 最小角と最大角の間のときはその角度になる
  goal.trajectory.points[0].positions[0] = (kMaxFingerDegrees - kMinFingerDegrees) / 2.0 * M_PI / 180;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeFromStart
                                                        + kGoalTimeTolerance)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
            action_client.getResult()->error_code);
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithoutObject(joint_state, (kMaxFingerDegrees - kMinFingerDegrees) / 2.0 * M_PI / 180);

  // 最大角より大きいときは最大角になる
  goal.trajectory.points[0].positions[0] = (kMaxFingerDegrees + 10.0) * M_PI / 180;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeFromStart
                                                        + kGoalTimeTolerance)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
            action_client.getResult()->error_code);
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithoutObject(joint_state, kMaxFingerDegrees * M_PI / 180);

  // 許容誤差時間が設定されていない場合でも正しく指定された角度になる
  goal.goal_time_tolerance = ros::Duration(0.0);
  goal.trajectory.points[0].positions[0] = (kMaxFingerDegrees - kMinFingerDegrees) / 2.0 * M_PI / 180;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeFromStart
                                                        + kGoalTimeTolerance)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
            action_client.getResult()->error_code);
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithoutObject(joint_state, (kMaxFingerDegrees - kMinFingerDegrees) / 2.0 * M_PI / 180);

  // 複数の関節が存在するとき正しく選ぶ
  goal.trajectory.points[0].positions.resize(2);
  goal.trajectory.points[0].positions[0] = kMaxFingerDegrees * M_PI / 180;
  goal.trajectory.points[0].positions[1] = kMinFingerDegrees * M_PI / 180;
  goal.trajectory.joint_names.resize(2);
  goal.trajectory.joint_names[0] = "hand_motro_joint";
  goal.trajectory.joint_names[1] = "hand_motor_joint";
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeFromStart
                                                        + kGoalTimeTolerance)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
            action_client.getResult()->error_code);
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithoutObject(joint_state, kMinFingerDegrees * M_PI / 180);

  // 複数の目標が存在するとき後の方を実行
  goal.trajectory.joint_names.resize(1);
  goal.trajectory.joint_names[0] = "hand_motor_joint";
  goal.trajectory.points.resize(2);
  goal.trajectory.points[0].time_from_start = ros::Duration(kTimeFromStart);
  goal.trajectory.points[0].positions.resize(1);
  goal.trajectory.points[0].positions[0] = kMinFingerDegrees * M_PI / 180;
  goal.trajectory.points[1].time_from_start = ros::Duration(kTimeFromStart);
  goal.trajectory.points[1].positions.resize(1);
  goal.trajectory.points[1].positions[0] = kMaxFingerDegrees * M_PI / 180;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeFromStart
                                                        + kGoalTimeTolerance)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
            action_client.getResult()->error_code);
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithoutObject(joint_state, kMaxFingerDegrees * M_PI / 180);

  // goalが二度続けて発行されたとき後の方を実行
  goal.trajectory.points[1].positions[0] = (kMaxFingerDegrees - kMinFingerDegrees) / 2.0 * M_PI / 180;
  action_client.sendGoal(goal);
  goal.trajectory.points[1].positions[0] = kMinFingerDegrees * M_PI / 180;
  action_client.sendGoal(goal);

  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeFromStart
                                                        + kGoalTimeTolerance)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
            action_client.getResult()->error_code);
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                                joint_state));
  ExpectHandJointStateWithoutObject(joint_state, kMinFingerDegrees * M_PI / 180);
}

// gripperのFollowJointTrajectoryアクション異常動作テスト
TEST_F(HsrbGazeboPluginsTest, GripperFollowJointTrajectoryActionFailure) {
  SetHandState(kOpenHand);

  sensor_msgs::JointState joint_state;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      action_client("/hsrb/gripper_controller/follow_joint_trajectory", true);
  ASSERT_TRUE(action_client.waitForServer(ros::Duration(kTimeout)));
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.goal_time_tolerance = ros::Duration(kGoalTimeTolerance);
  goal.trajectory.joint_names.resize(1);
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(1);
  goal.trajectory.points[0].time_from_start = ros::Duration(kTimeFromStart);

  // 関節角度名が存在しないとき
  goal.trajectory.joint_names[0] = "hand_motro_joint";
  goal.trajectory.points[0].positions[0] = kMinFingerDegrees * M_PI / 180;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeFromStart
                                                        + kGoalTimeTolerance)));
  ros::Duration(1.0).sleep();
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
            action_client.getResult()->error_code);
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                                joint_state));
  ExpectHandJointStateWithoutObject(joint_state, kMaxFingerDegrees * M_PI / 180);

  // 物体を挟んで到達しないときタイムアウト
  SetModelPose("hsrb", 0.0, 0.0, 0.05, -0.50, 0.50, 0.50, 0.50);
  ros::Duration(1.0).sleep();

  goal.trajectory.joint_names[0] = "hand_motor_joint";
  action_client.sendGoal(goal);
  ASSERT_FALSE(action_client.waitForResult(ros::Duration(kTimeFromStart
                                                         + kGoalTimeTolerance)));
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeFromStart
                                                        + kGoalTimeTolerance
                                                        + 1.0)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  EXPECT_EQ(control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED,
            action_client.getResult()->error_code);
  EXPECT_TRUE(joint_state_sub.WaitForMessage(ros::Duration(1.0),
                                             joint_state));
  ExpectHandJointStateWithObject(joint_state);
}

// grasp_hackプラグインの基本動作テスト
TEST_F(HsrbGazeboPluginsTest, GraspHack) {
  SetHandState(kOpenHand);

  // 両指で物体Attach
  SetModelPose("hsrb", -0.5, 0.0, 0.2, 0.0, 1.0, 0.0, 0.0);
  SetHandState(kCloseHand);
  SetHandToGraspCheckPos();
  ExpectGraspState(true);

  // 吸引OFF->OFFでは物体Detachされない
  SetSuctionState(false);
  ExpectGraspState(true);

  // ハンドを開くと物体Detach
  SetHandState(kOpenHand);
  ExpectGraspState(false);

  // 吸引OFFの際は片指で物体Attachできない
  SuctionAction();
  ExpectSuctionState(false);
  ExpectGraspState(false);

  // 吸引ONの際は片指で物体Attachできる
  SetSuctionState(true);
  SetHandState(kOpenHand);
  SuctionAction();
  ExpectSuctionState(true);
  ExpectGraspState(true);

  // 吸引ON->OFFで物体Detach
  SetSuctionState(false);
  ExpectSuctionState(false);
  ExpectGraspState(false);
}

// suctionアクションの正常動作テスト
TEST_F(HsrbGazeboPluginsTest, SuctionActionSucceed) {
  SetSuctionState(false);
  actionlib::SimpleActionClient<tmc_suction::SuctionControlAction>
      action_client("/hsrb/suction_control", true);
  ASSERT_TRUE(action_client.waitForServer(ros::Duration(kTimeout)));
  tmc_suction::SuctionControlGoal goal;

  // 吸引ONでは片指で物体Attachされる
  goal.suction_on.data = true;
  goal.timeout = ros::Duration(1.0);
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeout)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());

  SetHandState(kOpenHand);
  SuctionAction();
  ExpectSuctionState(true);
  ExpectGraspState(true);

  // 吸引OFFでは物体Detachされる
  goal.suction_on.data = false;
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeout)));
  EXPECT_EQ("SUCCEEDED", action_client.getState().toString());
  ros::Duration(1.0).sleep();
  ExpectSuctionState(false);
  ExpectGraspState(false);
}

// suctionアクションの異常動作テスト
TEST_F(HsrbGazeboPluginsTest, SuctionActionFailure) {
  SetSuctionState(false);
  actionlib::SimpleActionClient<tmc_suction::SuctionControlAction>
      action_client("/hsrb/suction_control", true);
  ASSERT_TRUE(action_client.waitForServer(ros::Duration(kTimeout)));
  tmc_suction::SuctionControlGoal goal;

  // タイムアウト時間が負
  goal.suction_on.data = true;
  goal.timeout = -ros::Duration(1.0);
  action_client.sendGoal(goal);
  ASSERT_TRUE(action_client.waitForResult(ros::Duration(kTimeout)));
  EXPECT_EQ("REJECTED", action_client.getState().toString());

  SetHandState(kOpenHand);
  SuctionAction();
  ExpectSuctionState(false);
  ExpectGraspState(false);
}

}  // namespace hsrb_gazebo_plugins

int main(int argc, char **argv) {
  ros::init(argc, argv, "hsrb_gazebo_plugins_test");
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
