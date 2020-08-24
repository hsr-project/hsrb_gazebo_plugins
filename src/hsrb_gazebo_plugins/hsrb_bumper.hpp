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
#ifndef HSRB_GAZEBO_PLUGINS_HSRB_BUMPER_HPP_
#define HSRB_GAZEBO_PLUGINS_HSRB_BUMPER_HPP_

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <sys/time.h>

#include <std_msgs/Bool.h>

#include <tf/tf.h>

#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/TransportTypes.hh>

namespace hsrb_gazebo_plugins {
/// @brief A Bumper plugin
class HsrbBumper : public gazebo::SensorPlugin{
 public:
  /// Constructor
  HsrbBumper();

  /// Destructor
  virtual ~HsrbBumper();

  /// Load the SDF elements and initialize
  virtual void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf);

 private:
  /// Update the bumper data
  void OnContact();

  ros::NodeHandle* rosnode_;
  ros::Publisher contact_pub_;

  gazebo::sensors::ContactSensorPtr parent_sensor_;

  std::string bumper_topic_name_;
  std::string frame_name_;
  std::string robot_namespace_;

  ros::CallbackQueue contact_queue_;
  void ContactQueueThread();
  boost::thread callback_queue_thread_;

  gazebo::event::ConnectionPtr update_connection_;
};
}  // namespace hsrb_gazebo_plugins

#endif  // HSRB_GAZEBO_PLUGINS_HSRB_BUMPER_HPP_
