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
#include <rtt/Component.hpp>

FtSensorGazebo::FtSensorGazebo(std::string const& name)
    : TaskContext(name, RTT::TaskContext::PreOperational)
    , slow_buffer_size_(2)
    , fast_buffer_size_(100)
    , slow_buffer_index_(0)
    , fast_buffer_index_(0)
    , data_valid_(false)
{
    addProperty("joint_name", joint_name_);
    addProperty("transform_xyz", transform_xyz_);
    addProperty("transform_rpy", transform_rpy_);

    // Add required gazebo interfaces
    this->provides("gazebo")->addOperation("configure",&FtSensorGazebo::gazeboConfigureHook,this,RTT::ClientThread);
    this->provides("gazebo")->addOperation("update",&FtSensorGazebo::gazeboUpdateHook,this,RTT::ClientThread);

    this->ports()->addPort("FxGage0_OUTPORT", port_FxGage0_out_);
    this->ports()->addPort("FyGage1_OUTPORT", port_FyGage1_out_);
    this->ports()->addPort("FzGage2_OUTPORT", port_FzGage2_out_);
    this->ports()->addPort("TxGage3_OUTPORT", port_TxGage3_out_);
    this->ports()->addPort("TyGage4_OUTPORT", port_TyGage4_out_);
    this->ports()->addPort("TzGage5_OUTPORT", port_TzGage5_out_);
    this->ports()->addPort("StatusCode_OUTPORT", port_StatusCode_out_);
    this->ports()->addPort("SampleCounter_OUTPORT", port_SampleCounter_out_);

    this->ports()->addPort("Control1_INPORT", port_Control1_in_);
    this->ports()->addPort("Control2_INPORT", port_Control2_in_);

    slow_buffer_.resize(slow_buffer_size_);
    fast_buffer_.resize(fast_buffer_size_);
}

FtSensorGazebo::~FtSensorGazebo() {
}

ORO_LIST_COMPONENT_TYPE(FtSensorGazebo)

