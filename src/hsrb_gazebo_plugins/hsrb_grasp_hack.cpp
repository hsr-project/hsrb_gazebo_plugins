/*
 * Copyright (C) 2015 Toyota Motor Corporation
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and Confidential
 *
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "hsrb_grasp_hack.hpp"

#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

#include <gazebo/common/Events.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>

#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>


namespace {
/// 接触リンク-パームリンクの姿勢の前フレームとの差分のVariance閾値(Attach判定用)
const double kPoseDiffVarThreshold = 1e-2;
/// 接触リンク-パームリンクの姿勢の前フレームとの差分のMax閾値(Attach判定用)
const double kPoseDiffMaxThreshold = 1e-2;
/// Attach判定させるリンク重量の閾値(大きいと判定しない)
const double kMaxAttachWeight = 1.2;
/// springに力がかかっていないと判定する偏差の閾値[rad]
const double kSpringNoForce = 0.05;
/// 位置偏差のbuffer数
const uint32_t kNumDiffBuffer = 2;
}  // anonymous namespace

namespace hsrb_gazebo_plugins {

GZ_REGISTER_MODEL_PLUGIN(HsrbGraspHack);

HsrbGraspHack::HsrbGraspHack()
    : zero_count_(0),
      pos_count_(0),
      diff_index_(0),
      attached_(false),
      is_suction_on_(false) {
  diffs_.resize(kNumDiffBuffer);
  ResetDiffs();
  prev_update_time_ = gazebo::common::Time::GetWallTime();
  update_rate_ = gazebo::common::Time(0, gazebo::common::Time::SecToNano(0.75));
}

HsrbGraspHack::~HsrbGraspHack() {
  model_.reset();
  physics_.reset();
  world_.reset();
  connections_.clear();
}

// モデルとsdf情報を読み込み初期化
void HsrbGraspHack::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ = model;
  world_ = model_->GetWorld();
#if GAZEBO_MAJOR_VERSION >= 8
  physics_ = world_->Physics();
#else
  physics_ = world_->GetPhysicsEngine();
#endif

  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo_node_->Init(world_->Name());
#else
  gazebo_node_->Init(world_->GetName());
#endif

  name_ = sdf->Get<std::string>("name");
  revolute_joint_ = physics_->CreateJoint("revolute", model_);

  sdf::ElementPtr grasp_check = sdf->GetElement("grasp_check");
  min_contact_count_ = grasp_check->Get<unsigned int>("min_contact_count");
  attach_steps_ = grasp_check->Get<int>("attach_steps");
  detach_steps_ = grasp_check->Get<int>("detach_steps");
  if (grasp_check->HasElement("max_attach_weight")) {
    attach_weight_ = grasp_check->Get<double>("max_attach_weight");
  } else {
    attach_weight_ = kMaxAttachWeight;
  }
  if (grasp_check->HasElement("spring_no_force")) {
    spring_no_force_ = grasp_check->Get<double>("spring_no_force");
  } else {
    spring_no_force_ = kSpringNoForce;
  }

  sdf::ElementPtr palm_link_elem = sdf->GetElement("palm_link");
  palm_link_ = model_->GetLink(palm_link_elem->Get<std::string>());
  if (!palm_link_) {
    gzerr << "palm link [" << palm_link_elem->Get<std::string>()
          << "] not found!\n";
  }

  std::string left_spring_joint = "hand_l_spring_proximal_joint";
  std::string right_spring_joint = "hand_r_spring_proximal_joint";
  if (grasp_check->HasElement("left_spring")) {
    sdf::ElementPtr left_spring_joint_elem = sdf->GetElement("left_spring");
    left_spring_joint = left_spring_joint_elem->Get<std::string>();
  }
  if (grasp_check->HasElement("right_spring")) {
    sdf::ElementPtr right_spring_joint_elem = sdf->GetElement("right_spring");
    right_spring_joint = right_spring_joint_elem->Get<std::string>();
  }

  left_spring_ = model_->GetJoint(left_spring_joint);
  right_spring_ = model_->GetJoint(right_spring_joint);
  if (!left_spring_ || !right_spring_) {
    ROS_FATAL("Spring Joint was not found");
    return;
  }

  sdf::ElementPtr gripper_link_elem = sdf->GetElement("gripper_link");

  while (gripper_link_elem) {
    gazebo::physics::LinkPtr gripper_link
        = model_->GetLink(gripper_link_elem->Get<std::string>());

    for (unsigned int j = 0; j < gripper_link->GetChildCount(); ++j) {
      gazebo::physics::CollisionPtr collision = gripper_link->GetCollision(j);
      std::map<std::string, gazebo::physics::CollisionPtr>::iterator coll_itr
          = collisions_.find(collision->GetScopedName());
      if (coll_itr != collisions_.end()) {
        continue;
      }

      collisions_[collision->GetScopedName()] = collision;
    }
    gripper_link_elem = gripper_link_elem->GetNextElement("gripper_link");
  }

  if (!collisions_.empty()) {
    // request the contact manager to publish messages to a custom topic for
    // this sensor
    gazebo::physics::ContactManager *contact_manager =
#if GAZEBO_MAJOR_VERSION >= 8
        world_->Physics()->GetContactManager();
#else
        world_->GetPhysicsEngine()->GetContactManager();
#endif
    std::string topic = contact_manager->CreateFilter(name(), collisions_);
    if (!contact_sub_) {
      contact_sub_ = gazebo_node_->Subscribe(topic,
                                      &HsrbGraspHack::OnContacts, this);
    }
  }
  connections_.push_back(gazebo::event::Events::ConnectWorldUpdateEnd(
      boost::bind(&HsrbGraspHack::OnUpdate, this)));

  std::string robot_namespace = "";
  if (sdf->HasElement("robot_namespace")) {
    robot_namespace = sdf->GetElement("robot_namespace")->Get<std::string>();
  } else {
    // ネームスペースが指定されない場合、Gazebo中のモデル名からつける。
    std::string name = model_->GetScopedName();
    std::vector<std::string> splitted;
    boost::split(splitted, name, boost::is_any_of("::"));
    robot_namespace = splitted.back();
  }
  InitNode(robot_namespace);
}

// ROS関係の初期化
void HsrbGraspHack::InitNode(const std::string& robot_namespace) {
  ros_node_.reset(new ros::NodeHandle(robot_namespace));
  suction_state_pub_ = ros_node_->advertise<std_msgs::Bool>(
      "pressure_sensor", 1);
  suction_command_sub_ = ros_node_->subscribe("command_suction",
                                              1,
                                              &HsrbGraspHack::SuctionCommandCB,
                                              this);
  suction_control_action_server_.reset(
      new SuctionControlActionServer(
          *ros_node_,
          "suction_control",
          boost::bind(&HsrbGraspHack::SuctionControlActionGoalCB, this, _1),
          false));
  suction_control_action_server_->start();
}

// メインループ
void HsrbGraspHack::OnUpdate() {
  if (gazebo::common::Time::GetWallTime() - prev_update_time_ < update_rate_) {
    return;
  }

  // @todo: should package the decision into a function
  if ((contacts_.size() >= min_contact_count_) &&
      (IsSpringLoaded() || is_suction_on_)) {
    pos_count_++;
    zero_count_ = 0;
  } else {
    zero_count_++;
    pos_count_ = std::max(0, pos_count_-1);
  }

  if (pos_count_ > attach_steps_ && !attached_) {
    HandleAttach();
  } else if (zero_count_ > detach_steps_ && attached_) {
    HandleDetach();
  }

  if (mutex_contacts_.try_lock()) {
    contacts_.clear();
    mutex_contacts_.unlock();
  }

  prev_update_time_ = gazebo::common::Time::GetWallTime();
  std_msgs::Bool suction_state_msg;
  suction_state_msg.data = is_suction_on_;
  suction_state_pub_.publish(suction_state_msg);
}

// check spring load if not suction on.
bool HsrbGraspHack::IsSpringLoaded() const {
#if GAZEBO_MAJOR_VERSION >= 8
  return ((fabs(left_spring_->Position(0)) > spring_no_force_) ||
          (fabs(right_spring_->Position(0)) > spring_no_force_));
#else
  return ((fabs(left_spring_->GetAngle(0).Radian()) > spring_no_force_) ||
          (fabs(right_spring_->GetAngle(0).Radian()) > spring_no_force_));
#endif
}

/// attach処理
void HsrbGraspHack::HandleAttach() {
  if (!palm_link_) {
    gzwarn << "palm link not found, not enforcing grasp hack!\n";
    return;
  }

  std::map<std::string, gazebo::physics::CollisionPtr> contact_collisions;
  std::map<std::string, int> contact_counts;
  std::map<std::string, int>::iterator iter;

  // This function is only called from the OnUpdate function so
  // the call to contacts.clear() is not going to happen in
  // parallel with the reads in the following code, no mutex
  // needed.
  for (unsigned int i = 0; i < contacts_.size(); ++i) {
    std::string name1 = contacts_[i].collision1();
    std::string name2 = contacts_[i].collision2();
    // 接触リンク名を重複なしで保存
    collide_links_.insert(name1);
    collide_links_.insert(name2);
    if (collisions_.find(name1) == collisions_.end()) {
      contact_collisions[name1] = boost::dynamic_pointer_cast<gazebo::physics::Collision>(
#if GAZEBO_MAJOR_VERSION >= 8
          world_->EntityByName(name1));
#else
          world_->GetEntity(name1));
#endif
      contact_counts[name1] += 1;
    }

    if (collisions_.find(name2) == collisions_.end()) {
      contact_collisions[name2] = boost::dynamic_pointer_cast<gazebo::physics::Collision>(
#if GAZEBO_MAJOR_VERSION >= 8
          world_->EntityByName(name2));
#else
          world_->GetEntity(name2));
#endif
      contact_counts[name2] += 1;
    }
  }
  for (iter = contact_counts.begin();
       iter != contact_counts.end();
       iter++) {
    if (iter->second < 2) {
      contact_counts.erase(iter++);
    } else {
      if (!attached_ && contact_collisions[iter->first]) {
        std::vector<gazebo::physics::LinkPtr> links = contact_collisions[iter->first]->GetModel()->GetLinks();
        double model_weight = 0.0;
        for (std::size_t i = 0; i < links.size(); ++i) {
#if GAZEBO_MAJOR_VERSION >= 8
          model_weight += links[i]->GetInertial()->Mass();
#else
          model_weight += links[i]->GetInertial()->GetMass();
#endif
        }
        if (model_weight > attach_weight_) {
          return;
        }
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d diff = contact_collisions[iter->first]->GetLink()->WorldPose() -
            palm_link_->WorldPose();
        double x = (diff - prev_diff_).Pos().X();
        double y = (diff - prev_diff_).Pos().Y();
        double z = (diff - prev_diff_).Pos().Z();
        double dd = x * x + y * y + z * z;
#else
        gazebo::math::Pose diff = contact_collisions[iter->first]->GetLink()->GetWorldPose() -
            palm_link_->GetWorldPose();
        double dd = (diff - prev_diff_).pos.GetSquaredLength();
#endif

        prev_diff_ = diff;

        diffs_[diff_index_] = dd;
#if GAZEBO_MAJOR_VERSION >= 8
        double var = ignition::math::variance<double>(diffs_);
        double max = ignition::math::max<double>(diffs_);
#else
        double var = gazebo::math::variance<double>(diffs_);
        double max = gazebo::math::max<double>(diffs_);
#endif

        if (var < kPoseDiffVarThreshold && max < kPoseDiffMaxThreshold) {
          // 吸引ONなら1リンクの接触でattach
          // 吸引OFFなら2リンクの接触でattach
          if (is_suction_on_ || collide_links_.size() > 2) {
            attached_ = true;

            revolute_joint_->Load(palm_link_,
#if GAZEBO_MAJOR_VERSION >= 8
                               contact_collisions[iter->first]->GetLink(), ignition::math::Pose3d());
#else
                               contact_collisions[iter->first]->GetLink(), gazebo::math::Pose());
#endif
            revolute_joint_->Init();
#if GAZEBO_MAJOR_VERSION >= 8
            revolute_joint_->SetUpperLimit(0, 0);
            revolute_joint_->SetLowerLimit(0, 0);
#else
            revolute_joint_->SetHighStop(0, 0);
            revolute_joint_->SetLowStop(0, 0);
#endif
          }
        }
        diff_index_ = (diff_index_ + 1) % kNumDiffBuffer;
      }
    }
  }
}

/// detach処理
void HsrbGraspHack::HandleDetach() {
  collide_links_.clear();
  attached_ = false;
  revolute_joint_->Detach();
}

/// 接触情報を取得
void HsrbGraspHack::OnContacts(ConstContactsPtr &_msg) {
  for (int i = 0; i < _msg->contact_size(); ++i) {
    gazebo::physics::CollisionPtr collision1 =
        boost::dynamic_pointer_cast<gazebo::physics::Collision>(
#if GAZEBO_MAJOR_VERSION >= 8
            world_->EntityByName(_msg->contact(i).collision1()));
#else
            world_->GetEntity(_msg->contact(i).collision1()));
#endif
    gazebo::physics::CollisionPtr collision2 =
        boost::dynamic_pointer_cast<gazebo::physics::Collision>(
#if GAZEBO_MAJOR_VERSION >= 8
            world_->EntityByName(_msg->contact(i).collision2()));
#else
            world_->GetEntity(_msg->contact(i).collision2()));
#endif

    if ((collision1 && !collision1->IsStatic()) &&
        (collision2 && !collision2->IsStatic())) {
      boost::mutex::scoped_lock lock(mutex_contacts_);
      contacts_.push_back(_msg->contact(i));
    }
  }
}

/// diffsの初期化処理
void HsrbGraspHack::ResetDiffs() {
  for (unsigned int i = 0; i < kNumDiffBuffer; ++i)
#if GAZEBO_MAJOR_VERSION >= 8
    diffs_[i] = IGN_DBL_MAX;
#else
    diffs_[i] = GZ_DBL_MAX;
#endif
}

/// 吸引コマンドのコールバック
void HsrbGraspHack::SuctionCommandCB(std_msgs::Bool::ConstPtr msg) {
  if (msg->data) {
    is_suction_on_ = true;
  } else {
    is_suction_on_ = false;
    HandleDetach();
  }
}

/// 吸引アクションGoalのコールバック
void HsrbGraspHack::SuctionControlActionGoalCB(SuctionControlActionGoalHandle goal) {
  tmc_suction::SuctionControlResult result;
  // 吸引開始アクション
  if (goal.getGoal()->suction_on.data) {
    // durationの負値が来たらリジェクト
    if (goal.getGoal()->timeout.toSec() < 0.0) {
      goal.setRejected(result);
      return;
    }
    goal.setAccepted();
    is_suction_on_ = true;
    goal.setSucceeded(result);
  } else {
    goal.setAccepted();
    if (is_suction_on_) {
      is_suction_on_ = false;
      HandleDetach();
    }
    goal.setSucceeded(result);
  }
}

}  // namespace hsrb_gazebo_plugins
