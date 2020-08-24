/*
Copyright (c) 2018 TOYOTA MOTOR CORPORATION
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

#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace hsrb_gazebo_plugins {

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
    while ((!is_sub_) &&
           ((ros::Time::now() - start) < timeout)) {
      ros::spinOnce();
      rate.sleep();
    }
    if (!is_sub_) {
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


class HsrbGazeboBumperPluginTest : public ::testing::Test {
 protected:
  HsrbGazeboBumperPluginTest() {
    ros::NodeHandle node;
    f_bumper_sub.Init("/hsrb/base_f_bumper_sensor");
    b_bumper_sub.Init("/hsrb/base_b_bumper_sensor");
  }

  virtual ~HsrbGazeboBumperPluginTest() {}

  virtual void SetUp() {
    ASSERT_TRUE(f_bumper_sub.WaitUntilConnectionEstablished(
        ros::Duration(10.0)));
    ASSERT_TRUE(b_bumper_sub.WaitUntilConnectionEstablished(
        ros::Duration(10.0)));
  }

  void SetModelPose(const std::string& model_name,
                    double position_x, double position_y, double position_z,
                    double orientation_x, double orientation_y,
                    double orientation_z, double orientation_w) {
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
    ros::Duration(1.0).sleep();
  }

  //バンパの状態チェック
  void ExpectBumperState(bool f_bumper_on, bool b_bumper_on) {
    std_msgs::Bool f_bumper_state;
    std_msgs::Bool b_bumper_state;
    EXPECT_TRUE(f_bumper_sub.WaitForMessage(ros::Duration(5.0),
                                            f_bumper_state));
    EXPECT_TRUE(b_bumper_sub.WaitForMessage(ros::Duration(5.0),
                                            b_bumper_state));
    ros::Time start = ros::Time::now();
    ros::Rate rate(10.0);
    while ((ros::Time::now() - start) < ros::Duration(3.0)) {
      EXPECT_EQ(f_bumper_on, f_bumper_state.data);
      EXPECT_EQ(b_bumper_on, b_bumper_state.data);
      rate.sleep();
     }
  }

  TestSubscriber<std_msgs::Bool> f_bumper_sub;
  TestSubscriber<std_msgs::Bool> b_bumper_sub;

 private:
  gazebo_msgs::SetModelState set_model_state_srv_;
};

// bumperプラグインの基本動作テスト
TEST_F(HsrbGazeboBumperPluginTest, Bumper) {
  // 何も当たっていない
  ExpectBumperState(false, false);

  // 前側に物体が当たる
  SetModelPose("wall_f", 0.235, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  ExpectBumperState(true, false);

  // 前側に物体が当たらなくなる
  SetModelPose("wall_f", 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  ExpectBumperState(false, false);

  // 後側に物体が当たる
  SetModelPose("wall_b", -0.235, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  ExpectBumperState(false, true);

  // 後側に物体が当たらなくなる
  SetModelPose("wall_b", -0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  ExpectBumperState(false, false);

  // 両側に物体が当たる
  SetModelPose("wall_f", 0.235, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  SetModelPose("wall_b", -0.235, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  ExpectBumperState(true, true);
}

}  // namespace hsrb_gazebo_plugins

int main(int argc, char **argv) {
  ros::init(argc, argv, "hsrb_bumper_plugin_test");
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
