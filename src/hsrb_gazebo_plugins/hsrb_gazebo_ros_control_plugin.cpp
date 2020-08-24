/// @brief gazebo_ros_controlプラグインのバグ修正版(本家へマージ予定)
/*
Copyright (c) 2017 TOYOTA MOTOR CORPORATION
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
#include <boost/bind.hpp>
#include "hsrb_gazebo_ros_control_plugin.hpp"

namespace hsrb_gazebo_plugins {

void HsrbGazeboRosControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) {
  GazeboRosControlPlugin::Load(parent, sdf);
  ROS_INFO_STREAM_NAMED("hsrb_gazebo_ros_control", "Loading hsrb_gazebo_ros_control plugin");
  // Reconnect the world update event
#if GAZEBO_MAJOR_VERSION >= 8
  update_connection_.reset();
#else
  gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
#endif
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&HsrbGazeboRosControlPlugin::Update,
                                                                                  this));
}

void HsrbGazeboRosControlPlugin::Update() {
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->SimTime();
#else
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
#endif
  ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);

  // This is only the difference from the original gazebo_ros_control
  // We call the readSim() function before the every update loop
  robot_hw_sim_->readSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);

  GazeboRosControlPlugin::Update();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HsrbGazeboRosControlPlugin);

}  // namespace hsrb_gazebo_plugins
