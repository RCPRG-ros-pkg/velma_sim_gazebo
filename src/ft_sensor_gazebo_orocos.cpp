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

#include "ft_sensor_gazebo.h"
#include <rtt/Logger.hpp>

using namespace RTT;

void FtSensorGazebo::updateHook() {
    // Synchronize with gazeboUpdate()
    RTT::os::MutexLock lock(gazebo_mutex_);

    if (!data_valid_) {
        return;
    }

    uint32_t Control1 = 0, Control2 = 0;
    port_Control1_in_.read(Control1);
    port_Control2_in_.read(Control2);

    port_FxGage0_out_.write(FxGage0_out_);
    port_FyGage1_out_.write(FyGage1_out_);
    port_FzGage2_out_.write(FzGage2_out_);
    port_TxGage3_out_.write(TxGage3_out_);
    port_TyGage4_out_.write(TyGage4_out_);
    port_TzGage5_out_.write(TzGage5_out_);

    StatusCode_out_ = 0;    // TODO
    port_StatusCode_out_.write(StatusCode_out_);
    port_SampleCounter_out_.write(SampleCounter_out_);
    ++SampleCounter_out_;
}

bool FtSensorGazebo::startHook() {
    return true;
}

bool FtSensorGazebo::configureHook() {
    Logger::In in("FtSensorGazebo::configureHook");

    joint_ = model_->GetJoint(joint_name_);
    if (joint_.get() == NULL) {
        Logger::log() << Logger::Error << "could not find joint \"" << joint_name_ << "\"" << Logger::endl;
        return false;
    }

    link_ = joint_->GetJointLink(0);

    if (transform_xyz_.size() != 3) {
        Logger::log() << Logger::Error << "wrong transform_xyz: vector size is " << transform_xyz_.size() << ", should be 3" << Logger::endl;
        return false;
    }

    if (transform_rpy_.size() != 3) {
        Logger::log() << Logger::Error << "wrong transform_rpy: vector size is " << transform_rpy_.size() << ", should be 3" << Logger::endl;
        return false;
    }

    T_W_S_ = KDL::Frame(KDL::Rotation::RPY(transform_rpy_[0], transform_rpy_[1], transform_rpy_[2]), KDL::Vector(transform_xyz_[0], transform_xyz_[1], transform_xyz_[2]));

    return true;
}

