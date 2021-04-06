/// @brief HSR-B用把持判定プラグイン
/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 * Copyright (C) 2015 TOYOTA MOTOR CORPORATION
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef HSRB_GAZEBO_PLUGINS_HSRB_GRASP_HACK_HPP_
#define HSRB_GAZEBO_PLUGINS_HSRB_GRASP_HACK_HPP_

#include <map>
#include <set>
#include <string>
#include <vector>

#include <boost/scoped_ptr.hpp>

#include <gazebo/common/Plugin.hh>
#if GAZEBO_MAJOR_VERSION < 8
#include <gazebo/math/Pose.hh>
#endif
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <actionlib/server/action_server.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tmc_suction/SuctionControlAction.h>

#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif

namespace hsrb_gazebo_plugins {

/// @class HsrbGraspHack
/// @brief HSR-B把持・吸引判定用プラグイン
class HsrbGraspHack : public gazebo::ModelPlugin {
 public:
  HsrbGraspHack();

  virtual ~HsrbGraspHack();
  /// モデルとsdf情報を読み込み初期化
  /// @param [in] model gazeboモデル情報
  /// @param [in] sdf sdf要素データ
  virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  /// 使用ハンド名を取得
  std::string name() const {
    return name_;
  }
  /// attach情報を取得(true: attach状態 false: detach状態)
  bool attached() const {
    return attached_;
  }

 private:
  typedef actionlib::ActionServer<tmc_suction::SuctionControlAction> SuctionControlActionServer;
  typedef boost::shared_ptr<SuctionControlActionServer> SuctionControlActionServerPtr;
  typedef SuctionControlActionServer::GoalHandle SuctionControlActionGoalHandle;

  /// メインループ
  void OnUpdate();
  /// 接触情報を取得
  /// @param [in] msg 接触情報
  void OnContacts(ConstContactsPtr &_msg);
  /// attach処理
  void HandleAttach();
  /// detach処理
  void HandleDetach();
  /// diffsの初期化処理
  void ResetDiffs();
  /// 力がかかっているか判定
  bool IsSpringLoaded() const;
  /// 力がかかり続けているかを最大の力との比率で判定する
  bool IsSpringLoadedFromRatio();

  /// ROS関係の初期化
  /// @param [in] robot_namespace ネームスペース
  void InitNode(const std::string& robot_namespace);
  /// 吸引アクションGoalのコールバック
  /// @param [in] goal アクションのGoalメッセージ
  void SuctionControlActionGoalCB(SuctionControlActionGoalHandle goal);
  /// 吸引コマンドのコールバック
  /// @param [in] msg 吸引コマンドのメッセージ
  void SuctionCommandCB(std_msgs::Bool::ConstPtr msg);

  gazebo::physics::ModelPtr model_;
  gazebo::physics::PhysicsEnginePtr physics_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::JointPtr revolute_joint_;
  gazebo::physics::JointPtr left_spring_;
  gazebo::physics::JointPtr right_spring_;
  gazebo::physics::LinkPtr palm_link_;
  std::vector<gazebo::event::ConnectionPtr> connections_;
  std::map<std::string, gazebo::physics::CollisionPtr> collisions_;
  std::vector<gazebo::msgs::Contact> contacts_;
  boost::mutex mutex_contacts_;
  bool attached_;
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d prev_diff_;
#else
  gazebo::math::Pose prev_diff_;
#endif
  std::vector<double> diffs_;
  int diff_index_;
  gazebo::common::Time update_rate_;
  gazebo::common::Time prev_update_time_;
  int pos_count_;
  int zero_count_;
  unsigned int min_contact_count_;
  int attach_steps_;
  int detach_steps_;
  double attach_weight_;
  double spring_no_force_;
  double spring_no_force_ratio_;
  double spring_max_force_l_;
  double spring_max_force_r_;
  std::string name_;
  gazebo::transport::SubscriberPtr contact_sub_;

  /// ROSノードハンドル
  boost::scoped_ptr<ros::NodeHandle> ros_node_;
  /// 吸引状態を読む
  ros::Publisher suction_state_pub_;
  /// 吸引コマンドを読む
  ros::Subscriber suction_command_sub_;
  /// 吸引アクションサーバー
  SuctionControlActionServerPtr suction_control_action_server_;
  /// 吸引状態 (true:吸引作動中 false:吸引停止中)
  bool is_suction_on_;
  /// 接触中のリンク名
  std::set<std::string> collide_links_;

 protected:
  gazebo::transport::NodePtr gazebo_node_;
};

}  // namespace hsrb_gazebo_plugins

#endif  // HSRB_GAZEBO_PLUGINS_HSRB_GRASP_HACK_HPP_
