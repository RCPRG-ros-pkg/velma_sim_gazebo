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

#include "torso_gazebo.h"
#include <rtt/Component.hpp>

using namespace controller_common::elmo_servo;

TorsoGazebo::TorsoGazebo(std::string const& name)
    : TaskContext(name, RTT::TaskContext::PreOperational)
    , data_valid_(false)
    , port_t_MotorPosition_out_("t_MotorPosition_OUTPORT", false)
    , port_t_MotorVelocity_out_("t_MotorVelocity_OUTPORT", false)
    , port_hp_q_out_("head_pan_motor_position_OUTPORT", false)
    , port_hp_v_out_("head_pan_motor_velocity_OUTPORT", false)
    , port_ht_q_out_("head_tilt_motor_position_OUTPORT", false)
    , port_ht_v_out_("head_tilt_motor_velocity_OUTPORT", false)
    , hp_homing_done_(false)
    , hp_homing_in_progress_(false)
    , ht_homing_done_(false)
    , ht_homing_in_progress_(false)
    , t_servo_state_(ServoState::NOT_READY_TO_SWITCH_ON)
    , hp_servo_state_(ServoState::NOT_READY_TO_SWITCH_ON)
    , ht_servo_state_(ServoState::NOT_READY_TO_SWITCH_ON)
    , kinect_active_(false)
    , first_step_(true)
{
    // Add required gazebo interfaces
    this->provides("gazebo")->addOperation("configure",&TorsoGazebo::gazeboConfigureHook,this,RTT::ClientThread);
    this->provides("gazebo")->addOperation("update",&TorsoGazebo::gazeboUpdateHook,this,RTT::ClientThread);

    // torso ports
    this->ports()->addPort("t_MotorCurrentCommand_INPORT",      port_t_MotorCurrentCommand_in_);
    this->ports()->addPort("t_MotorControlWord_INPORT",         port_t_MotorControlWord_in_);
    this->ports()->addPort(port_t_MotorPosition_out_);
    this->ports()->addPort(port_t_MotorVelocity_out_);
    this->ports()->addPort("t_MotorStatus_OUTPORT", port_t_MotorStatus_out_);
    t_MotorCurrentCommand_in_ = 0.0;

    // head ports
    this->ports()->addPort("head_pan_motor_position_command_INPORT",        port_hp_q_in_);
    this->ports()->addPort("head_pan_motor_velocity_command_INPORT",        port_hp_v_in_);
    this->ports()->addPort("head_pan_motor_current_command_INPORT",         port_hp_c_in_);
    this->ports()->addPort("head_pan_motor_controlWord_INPORT",             port_hp_controlWord_in_);
    this->ports()->addPort(port_hp_q_out_);
    this->ports()->addPort(port_hp_v_out_);
    this->ports()->addPort("head_pan_motor_status_OUTPORT", port_hp_status_out_);
    hp_q_in_ = hp_v_in_ = hp_c_in_ = hp_q_out_ = hp_v_out_ = 0.0;
    this->ports()->addPort("head_tilt_motor_position_command_INPORT",       port_ht_q_in_);
    this->ports()->addPort("head_tilt_motor_velocity_command_INPORT",       port_ht_v_in_);
    this->ports()->addPort("head_tilt_motor_current_command_INPORT",        port_ht_c_in_);
    this->ports()->addPort("head_tilt_motor_controlWord_INPORT",            port_ht_controlWord_in_);
    this->ports()->addPort(port_ht_q_out_);
    this->ports()->addPort(port_ht_v_out_);
    this->ports()->addPort("head_tilt_motor_status_OUTPORT", port_ht_status_out_);
    ht_q_in_ = ht_v_in_ = ht_c_in_ = ht_q_out_ = ht_v_out_ = 0.0;

    addProperty("hp_kp", hp_kp_);
    addProperty("hp_ki", hp_ki_);
    addProperty("hp_kd", hp_kd_);
    addProperty("hp_min_i", hp_min_i_);
    addProperty("hp_max_i", hp_max_i_);
    addProperty("hp_min_cmd", hp_min_cmd_);
    addProperty("hp_max_cmd", hp_max_cmd_);

    addProperty("ht_kp", ht_kp_);
    addProperty("ht_ki", ht_ki_);
    addProperty("ht_kd", ht_kd_);
    addProperty("ht_min_i", ht_min_i_);
    addProperty("ht_max_i", ht_max_i_);
    addProperty("ht_min_cmd", ht_min_cmd_);
    addProperty("ht_max_cmd", ht_max_cmd_);

}

TorsoGazebo::~TorsoGazebo() {
}

ORO_LIST_COMPONENT_TYPE(TorsoGazebo)

