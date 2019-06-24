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

#ifndef TORSO_GAZEBO_H__
#define TORSO_GAZEBO_H__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "Eigen/Dense"

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>

#include "rtt_rosclock/rtt_rosclock.h"

#include <controller_common/elmo_servo_state.h>

class TorsoGazebo : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // torso ports
    RTT::InputPort<int16_t >    port_t_MotorCurrentCommand_in_;
    RTT::InputPort<uint16_t >   port_t_MotorControlWord_in_;
    RTT::OutputPort<int32_t >   port_t_MotorPosition_out_;
    RTT::OutputPort<int32_t >   port_t_MotorVelocity_out_;
    RTT::OutputPort<uint16_t >  port_t_MotorStatus_out_;

    int16_t t_MotorCurrentCommand_in_;
    int32_t t_MotorPosition_out_;
    int32_t t_MotorVelocity_out_;

    // head ports
    RTT::InputPort<int32_t>      port_hp_q_in_;
    RTT::InputPort<int32_t>      port_hp_v_in_;
    RTT::InputPort<int32_t>      port_hp_c_in_;
    RTT::InputPort<uint16_t >    port_hp_controlWord_in_;
    RTT::OutputPort<int32_t>     port_hp_q_out_;
    RTT::OutputPort<int32_t>     port_hp_v_out_;
    RTT::OutputPort<uint16_t >   port_hp_status_out_;

    RTT::InputPort<int32_t>      port_ht_q_in_;
    RTT::InputPort<int32_t>      port_ht_v_in_;
    RTT::InputPort<int32_t>      port_ht_c_in_;
    RTT::InputPort<uint16_t >    port_ht_controlWord_in_;
    RTT::OutputPort<int32_t>     port_ht_q_out_;
    RTT::OutputPort<int32_t>     port_ht_v_out_;
    RTT::OutputPort<uint16_t >   port_ht_status_out_;

    int32_t hp_q_in_;
    int32_t hp_v_in_;
    int32_t hp_c_in_;
    int32_t hp_q_out_;
    int32_t hp_v_out_;

    int32_t ht_q_in_;
    int32_t ht_v_in_;
    int32_t ht_c_in_;
    int32_t ht_q_out_;
    int32_t ht_v_out_;

    bool hp_homing_done_;
    bool hp_homing_in_progress_;

    bool ht_homing_done_;
    bool ht_homing_in_progress_;

    controller_common::elmo_servo::ServoState t_servo_state_;
    controller_common::elmo_servo::ServoState hp_servo_state_;
    controller_common::elmo_servo::ServoState ht_servo_state_;

    // public methods
    TorsoGazebo(std::string const& name);
    ~TorsoGazebo();
    void updateHook();
    bool startHook();
    bool configureHook();
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  protected:

    controller_common::elmo_servo::ServoState getNextServoState(controller_common::elmo_servo::ServoState current_state, uint16_t controlWord) const;

    typedef Eigen::Matrix<double, 2, 1 > HeadJoints;

    int16_t tmp_t_MotorCurrentCommand_in_;
    int32_t tmp_t_MotorPosition_out_;
    int32_t tmp_t_MotorVelocity_out_;

    int32_t tmp_hp_q_in_;
    int32_t tmp_hp_v_in_;
    int32_t tmp_hp_c_in_;
    int32_t tmp_hp_q_out_;
    int32_t tmp_hp_v_out_;

    int32_t tmp_ht_q_in_;
    int32_t tmp_ht_v_in_;
    int32_t tmp_ht_c_in_;
    int32_t tmp_ht_q_out_;
    int32_t tmp_ht_v_out_;

    void setJointsPID();

    gazebo::physics::ModelPtr model_;

    // head
    gazebo::physics::JointPtr torso_joint_;
    gazebo::physics::JointPtr head_pan_joint_;
    gazebo::physics::JointPtr head_tilt_joint_;

    std::string head_pan_scoped_name_;
    std::string head_tilt_scoped_name_;

    gazebo::physics::JointController *jc_;

    void getJointPositionAndVelocity(double &q, double &dq);
    void getHeadJointPositionAndVelocity(HeadJoints &q, HeadJoints &dq);
    void setForces(double t);

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    bool data_valid_;

    ros::Time last_update_time_;

    bool kinect_active_;
    bool first_step_;
};

#endif  // TORSO_GAZEBO_H__

