/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef BARRETT_TACTILE_GAZEBO_H__
#define BARRETT_TACTILE_GAZEBO_H__

#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include <barrett_hand_status_msgs/BHPressureState.h>
//#include <barrett_hand_msgs/BHTemp.h>
#include <barrett_hand_status_msgs/BHPressureInfo.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "Eigen/Dense"

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>

#include "barrett_hand_tactile/tactile.h"

class BarrettTactileGazebo : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // public methods
    BarrettTactileGazebo(std::string const& name);
    ~BarrettTactileGazebo();
    void updateHook();
    bool startHook();
    bool configureHook();
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  protected:

    std::string prefix_;

    int32_t median_filter_samples_, median_filter_max_samples_;

    gazebo::physics::ModelPtr model_;
    bool data_valid_;

    std::vector<std::string > link_names_;
    std::vector<Eigen::Isometry3d > vec_T_C_L_;

    Tactile *ts_[4];

    // OROCOS ports
    RTT::OutputPort<barrett_hand_status_msgs::BHPressureState> port_tactile_out_;
    RTT::InputPort<std_msgs::Empty> port_reset_in_;
    RTT::InputPort<std_msgs::Empty> port_calibrate_in_;
    RTT::InputPort<std_msgs::Int32> port_filter_in_;
    RTT::OutputPort<barrett_hand_status_msgs::BHPressureInfo> port_tactile_info_out_;
    RTT::OutputPort<Eigen::Vector4d > port_max_pressure_out_;

    // port variables
    barrett_hand_status_msgs::BHPressureState tactile_out_;
    std_msgs::Empty reset_in_;
    std_msgs::Empty calibrate_in_;
    std_msgs::Int32 filter_in_;
    barrett_hand_status_msgs::BHPressureInfo pressure_info_;
    Eigen::Vector4d max_pressure_out_;

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;
};

#endif  // BARRETT_TACTILE_GAZEBO_H__

