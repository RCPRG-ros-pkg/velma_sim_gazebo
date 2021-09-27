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
#include <rtt/Logger.hpp>

using namespace RTT;

using namespace controller_common::elmo_servo;

ServoState TorsoGazebo::getNextServoState(ServoState current_state, uint16_t controlWord) const {
    ServoState next_state;
    if ( current_state == ServoState::NOT_READY_TO_SWITCH_ON && (controlWord&0x000F) == 0x0000 ) {
        return ServoState::SWITCH_ON_DISABLED;
    }
    else if ( current_state == ServoState::SWITCH_ON_DISABLED && (controlWord&0x000F) == 0x0006 ) {
        return ServoState::READY_TO_SWITCH_ON;
    }
    else if ( current_state == ServoState::READY_TO_SWITCH_ON && (controlWord&0x000F) == 0x0007 ) {
        return ServoState::SWITCH_ON;
    }
    else if ( current_state == ServoState::SWITCH_ON && (controlWord&0x000F) == 0x000F ) {
        return ServoState::OPERATION_ENABLED;
    }
    else if ( current_state == ServoState::OPERATION_ENABLED && (controlWord&0x000F) == 0x0007 ) {
        return ServoState::SWITCH_ON;
    }
    return current_state;
}

void TorsoGazebo::updateHook() {

    // Synchronize with gazeboUpdate()
    RTT::os::MutexLock lock(gazebo_mutex_);

    if (!data_valid_) {
        //Logger::In in("TorsoGazebo::updateHook");
        //Logger::log() << Logger::Debug << "gazebo is not initialized" << Logger::endl;
        return;
    }
    else {
        //Logger::log() << Logger::Debug << Logger::endl;
    }

    port_t_MotorPosition_out_.write(t_MotorPosition_out_);
    port_t_MotorVelocity_out_.write(t_MotorVelocity_out_);

    uint16_t hp_controlWord_in;
    if (port_hp_controlWord_in_.read(hp_controlWord_in) == RTT::NewData) {
        if ( (hp_controlWord_in&0x10) != 0 && !hp_homing_in_progress_) {
            if (hp_homing_done_) {
                Logger::In in("TorsoGazebo::updateHook");
                Logger::log() << Logger::Warning << "Running homing second time for head pan motor!" << Logger::endl;
            }
            else {
                hp_homing_in_progress_ = true;
                Logger::log() << Logger::Info << "Running homing head pan motor" << Logger::endl;
            }
        }
        ServoState prev_state = hp_servo_state_;
        hp_servo_state_ = getNextServoState(hp_servo_state_, hp_controlWord_in);
        if (prev_state != hp_servo_state_) {
            Logger::In in("TorsoGazebo::updateHook");
            Logger::log() << Logger::Info << "hp motor state: " << getServoStateStr(hp_servo_state_) << Logger::endl;
        }
    }
    else {
        hp_servo_state_ = ServoState::NOT_READY_TO_SWITCH_ON;
    }

    uint16_t ht_controlWord_in;
    if (port_ht_controlWord_in_.read(ht_controlWord_in) == RTT::NewData) {
        if ( (ht_controlWord_in&0x10) != 0 && !ht_homing_in_progress_) {
            if (ht_homing_done_) {
                Logger::In in("TorsoGazebo::updateHook");
                Logger::log() << Logger::Warning << "Running homing second time for head tilt motor!" << Logger::endl;
            }
            else {
                ht_homing_in_progress_ = true;
                Logger::log() << Logger::Info << "Running homing head tilt motor" << Logger::endl;
            }
        }
        ServoState prev_state = ht_servo_state_;
        ht_servo_state_ = getNextServoState(ht_servo_state_, ht_controlWord_in);
        if (prev_state != ht_servo_state_) {
            Logger::In in("TorsoGazebo::updateHook");
            Logger::log() << Logger::Info << "ht motor state: " << getServoStateStr(ht_servo_state_) << Logger::endl;
        }
    }
    else {
        ht_servo_state_ = ServoState::NOT_READY_TO_SWITCH_ON;
    }

    uint16_t t_controlWord_in;
    if (port_t_MotorControlWord_in_.read(t_controlWord_in) == RTT::NewData) {
        ServoState prev_state = t_servo_state_;
        t_servo_state_ = getNextServoState(t_servo_state_, t_controlWord_in);
        if (prev_state != t_servo_state_) {
            Logger::In in("TorsoGazebo::updateHook");
            Logger::log() << Logger::Info << "t motor state: " << getServoStateStr(t_servo_state_) << Logger::endl;
        }
    }
    else {
        t_servo_state_ = ServoState::NOT_READY_TO_SWITCH_ON;
    }

    uint16_t t_status_out = getStatusWord(t_servo_state_);
    uint16_t hp_status_out = getStatusWord(hp_servo_state_);
    uint16_t ht_status_out = getStatusWord(ht_servo_state_);

    if (hp_homing_done_) {
        hp_status_out |= 0x1400;
    }
    if (ht_homing_done_) {
        ht_status_out |= 0x1400;
    }

    port_t_MotorStatus_out_.write(t_status_out);
    port_hp_status_out_.write(hp_status_out);
    port_ht_status_out_.write(ht_status_out);

    port_t_MotorCurrentCommand_in_.read(t_MotorCurrentCommand_in_);

    ros::Time now = rtt_rosclock::host_now();
    double cmd_div = std::max(1.0, (now - last_update_time_).toSec()/0.001);
    last_update_time_ = now;
    t_MotorCurrentCommand_in_ = t_MotorCurrentCommand_in_ / cmd_div;

    //
    // head
    //
    port_hp_q_in_.read(hp_q_in_);
    port_hp_v_in_.read(hp_v_in_);
    port_hp_c_in_.read(hp_c_in_);
    port_hp_q_out_.write(hp_q_out_);
    port_hp_v_out_.write(hp_v_out_);
    port_ht_q_in_.read(ht_q_in_);
    port_ht_v_in_.read(ht_v_in_);
    port_ht_c_in_.read(ht_c_in_);
    port_ht_q_out_.write(ht_q_out_);
    port_ht_v_out_.write(ht_v_out_);
}

bool TorsoGazebo::startHook() {
  return true;
}

bool TorsoGazebo::configureHook() {

    {
        gazebo::physics::JointPtr joint = model_->GetJoint("head_pan_joint");
        vjc_.push_back( VelmaJointController(joint, 0.002, hp_kp_, hp_ki_,
            hp_min_i_, hp_max_i_, hp_min_cmd_, hp_max_cmd_) );
    }

    {
        gazebo::physics::JointPtr joint = model_->GetJoint("head_tilt_joint");
        vjc_.push_back( VelmaJointController(joint, 0.002, ht_kp_, ht_ki_,
            ht_min_i_, ht_max_i_, ht_min_cmd_, ht_max_cmd_) );
    }


    return true;
}

