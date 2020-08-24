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
#include "hsrb_bumper.hpp"

#include <string>

#include <gazebo/common/Exception.hh>

#if GAZEBO_MAJOR_VERSION < 8
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/math/Vector3.hh>
#endif
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <sdf/Param.hh>

namespace hsrb_gazebo_plugins {
// Register this plugin with gazebo
GZ_REGISTER_SENSOR_PLUGIN(HsrbBumper);

// Constructor
HsrbBumper::HsrbBumper()
    : SensorPlugin() {
  rosnode_ = nullptr;
}

// Destructor
HsrbBumper::~HsrbBumper() {
  if (rosnode_) {
    rosnode_->shutdown();
  }
  callback_queue_thread_.join();

  delete rosnode_;
}

// Load the SDF and initialize
void HsrbBumper::Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf) {
  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  parent_sensor_ = dynamic_pointer_cast<gazebo::sensors::ContactSensor>(parent);
  if (!parent_sensor_) {
    ROS_ERROR("The bumper plugin's parent sensor is not of type ContactSensor.");
    return;
  }

  robot_namespace_ = "";
  if (sdf->HasElement("robotNamespace")) {
    robot_namespace_ =
      sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  bumper_topic_name_ = "bumper_states";

  if (sdf->HasElement("bumperTopicName")) {
    bumper_topic_name_ =
      sdf->GetElement("bumperTopicName")->Get<std::string>();
  }

  if (!sdf->HasElement("frameName")) {
    ROS_INFO("Bumper plugin does not have <frameName>, defaulting to world.");
    frame_name_ = "world";
  } else {
    frame_name_ = sdf->GetElement("frameName")->Get<std::string>();
  }

  // It is necessary to initialize the ROS node for Gazebo
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, failed to load plugin.");
    return;
  }

  rosnode_ = new ros::NodeHandle(robot_namespace_);

  std::string prefix;
  rosnode_->getParam(std::string("tf_prefix"), prefix);
  frame_name_ = tf::resolve(prefix, frame_name_);

  contact_pub_ = rosnode_->advertise<std_msgs::Bool>(
      bumper_topic_name_, 1);

  callback_queue_thread_ = boost::thread(
      boost::bind(&HsrbBumper::ContactQueueThread, this));

  update_connection_ = parent_sensor_->ConnectUpdated(
      boost::bind(&HsrbBumper::OnContact, this));

  parent_sensor_->SetActive(true);
}

// Update the bumper data
void HsrbBumper::OnContact() {
  if (contact_pub_.getNumSubscribers() <= 0)
    return;

  gazebo::msgs::Contacts contacts;
# if GAZEBO_MAJOR_VERSION >= 7
  contacts = parent_sensor_->Contacts();
# else
  contacts = parent_sensor_->GetContacts();
# endif
  std_msgs::Bool msg;
  msg.data = contacts.contact_size() > 0;
  contact_pub_.publish(msg);
}

// Custom Callback Queue
void HsrbBumper::ContactQueueThread() {
  static const double timeout = 0.01;

  while (rosnode_->ok()) {
    contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}  // namespace hsrb_gazebo_plugins
