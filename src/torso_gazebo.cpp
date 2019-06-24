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
#include "velma_sim_conversion.h"

#include <gazebo/sensors/sensors.hh>

using namespace RTT;

void TorsoGazebo::getJointPositionAndVelocity(double &q, double &dq) {
    q = torso_joint_->Position();
    dq = torso_joint_->GetVelocity(0);
}

void TorsoGazebo::getHeadJointPositionAndVelocity(TorsoGazebo::HeadJoints &q, TorsoGazebo::HeadJoints &dq) {
    q(0) = head_pan_joint_->Position();
    dq(0) = head_pan_joint_->GetVelocity(0);

    q(1) = head_tilt_joint_->Position();
    dq(1) = head_tilt_joint_->GetVelocity(0);
}

void TorsoGazebo::setForces(double t) {
    torso_joint_->SetForce(0, t);
}

bool TorsoGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
    Logger::In in("TorsoGazebo::gazeboConfigureHook");

    if(model.get() == NULL) {
        Logger::log() << Logger::Error << "gazebo model is NULL" << Logger::endl;
        return false;
    }

    model_ = model;

    jc_ = new gazebo::physics::JointController(model);

    torso_joint_ = model->GetJoint("torso_0_joint");

    // head joints
    head_pan_joint_ = model->GetJoint("head_pan_joint");
    head_tilt_joint_ = model->GetJoint("head_tilt_joint");

    head_pan_scoped_name_ = head_pan_joint_->GetScopedName();
    head_tilt_scoped_name_ = head_tilt_joint_->GetScopedName();

    return true;
}

void TorsoGazebo::setJointsPID() {
    jc_->Reset();

    jc_->AddJoint(head_pan_joint_);
    jc_->SetPositionPID(head_pan_scoped_name_, gazebo::common::PID(2.0, 1.0, 0.0, 0.5, -0.5, 10.0,-10.0));
    jc_->SetPositionTarget(head_pan_scoped_name_, head_pan_joint_->Position());

    jc_->AddJoint(head_tilt_joint_);
    jc_->SetPositionPID(head_tilt_scoped_name_, gazebo::common::PID(2.0, 1.0, 0.0, 0.5, -0.5, 10.0,-10.0));
    jc_->SetPositionTarget(head_tilt_scoped_name_, head_tilt_joint_->Position());
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void TorsoGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    Logger::In in("TorsoGazebo::gazeboUpdateHook");

    if (first_step_) {
        setJointsPID();
        first_step_ = false;
    }

    bool kinect_active_prev = kinect_active_;
    gazebo::sensors::SensorPtr kinect = gazebo::sensors::SensorManager::Instance()->GetSensor("openni_camera_camera");
    kinect_active_ = false;
    if (kinect && kinect->IsActive()) {
        kinect_active_ = true;
    }

    if (kinect_active_prev != kinect_active_) {
        if (kinect_active_) {
            Logger::log() << Logger::Info << "kinect is enabled" << Logger::endl;
        }
        else {
            Logger::log() << Logger::Info << "kinect is disabled" << Logger::endl;
        }
    }

    //
    // torso
    //
    double q_t, dq_t;
    getJointPositionAndVelocity(q_t, dq_t);

    const double torso_gear = 158.0;
    const double torso_trans_mult = 131072.0 * torso_gear / (M_PI * 2.0);
    const double torso_motor_offset = 270119630.0;
    const double torso_joint_offset = 0;
    const double torso_motor_constant = 0.00105;

    tmp_t_MotorPosition_out_ = (q_t - torso_joint_offset) * torso_trans_mult + torso_motor_offset;
    tmp_t_MotorVelocity_out_ = dq_t * torso_trans_mult;

    //
    // head
    //
    HeadJoints q_h, dq_h;
    getHeadJointPositionAndVelocity(q_h, dq_h);

    const double head_trans = 8000.0 * 100.0 / (M_PI * 2.0);
    tmp_hp_q_out_ = -q_h(0) * head_trans;
    tmp_ht_q_out_ = q_h(1) * head_trans;
    tmp_hp_v_out_ = dq_h(0);
    tmp_ht_v_out_ = dq_h(1);

    // exchange the data between Orocos and Gazebo
    {
        RTT::os::MutexLock lock(gazebo_mutex_);

        Logger::log() << Logger::Debug << Logger::endl;

        t_MotorPosition_out_ = tmp_t_MotorPosition_out_;
        t_MotorVelocity_out_ = tmp_t_MotorVelocity_out_;
        hp_q_out_ = tmp_hp_q_out_;
        ht_q_out_ = tmp_ht_q_out_;
        hp_v_out_ = tmp_hp_v_out_;
        ht_v_out_ = tmp_ht_v_out_;

        tmp_t_MotorCurrentCommand_in_ = t_MotorCurrentCommand_in_;
        tmp_hp_q_in_ = hp_q_in_;
        tmp_ht_q_in_ = ht_q_in_;

        data_valid_ = true;
    }

    double grav;
    grav = tmp_t_MotorCurrentCommand_in_ * torso_gear * torso_motor_constant;

    setForces(grav);

    // joint controller for the head
    if (hp_homing_in_progress_) {
        if (q_h(0) > 0.015) {
            jc_->SetPositionTarget(head_pan_scoped_name_, q_h(0)-0.008);
        }
        else if (q_h(0) < -0.015) {
            jc_->SetPositionTarget(head_pan_scoped_name_, q_h(0)+0.008);
        }
        else {
            hp_homing_in_progress_ = false;
            hp_homing_done_ = true;
        }
    }
    else if (hp_homing_done_) {
        jc_->SetPositionTarget(head_pan_scoped_name_, -tmp_hp_q_in_ / head_trans);
    }

    if (ht_homing_in_progress_) {
        if (q_h(1) > 0.015) {
            jc_->SetPositionTarget(head_tilt_scoped_name_, q_h(1)-0.008);
        }
        else if (q_h(1) < -0.015) {
            jc_->SetPositionTarget(head_tilt_scoped_name_, q_h(1)+0.008);
        }
        else {
            ht_homing_in_progress_ = false;
            ht_homing_done_ = true;
        }
    }
    else if (ht_homing_done_) {
        jc_->SetPositionTarget(head_tilt_scoped_name_, tmp_ht_q_in_ / head_trans);
    }

    jc_->Update();
}

