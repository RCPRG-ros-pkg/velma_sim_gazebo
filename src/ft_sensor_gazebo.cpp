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

void FtSensorGazebo::WrenchKDLToMsg(const KDL::Wrench &in,
                                    geometry_msgs::Wrench &out) const {
    out.force.x = in[0];
    out.force.y = in[1];
    out.force.z = in[2];
    out.torque.x = in[3];
    out.torque.y = in[4];
    out.torque.z = in[5];
}

bool FtSensorGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
    Logger::In in("FtSensorGazebo::gazeboConfigureHook");

    if(model.get() == NULL) {
        Logger::log() << Logger::Error << "gazebo model is NULL" << Logger::endl;
        return false;
    }

    model_ = model;
    
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void FtSensorGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    if (joint_.get() == NULL) {
        return;
    }
    ignition::math::Vector3d force = joint_->LinkForce(0);
    ignition::math::Vector3d torque = joint_->LinkTorque(0);

    KDL::Wrench wr_W = KDL::Wrench(-KDL::Vector(force.X(), force.Y(), force.Z()), -KDL::Vector(torque.X(), torque.Y(), torque.Z()));
    KDL::Wrench wr_S = (T_W_S_.Inverse() * wr_W);

    {
        RTT::os::MutexLock lock(gazebo_mutex_);
        // TODO: data conversion:
        double mult = 1000000.0;
        FxGage0_out_ = wr_S.force.x() * mult;
        FyGage1_out_ = wr_S.force.y() * mult;
        FzGage2_out_ = wr_S.force.z() * mult;
        TxGage3_out_ = wr_S.torque.x() * mult;
        TyGage4_out_ = wr_S.torque.y() * mult;
        TzGage5_out_ = wr_S.torque.z() * mult;
        data_valid_ = true;
    }
}

