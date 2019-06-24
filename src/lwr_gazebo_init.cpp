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

#include "lwr_gazebo.h"
#include <rtt/Component.hpp>

    LWRGazebo::LWRGazebo(std::string const& name)
        : TaskContext(name, RTT::TaskContext::PreOperational)
        , data_valid_(false)
        , port_CartesianWrench_out_("CartesianWrench_OUTPORT", false)
        , port_RobotState_out_("RobotState_OUTPORT", false)
        , port_FRIState_out_("FRIState_OUTPORT", false)
        , port_JointVelocity_out_("JointVelocity_OUTPORT", false)
        , port_MassMatrix_out_("MassMatrix_OUTPORT", false)
        , port_JointTorque_out_("JointTorque_OUTPORT", false)
        , port_GravityTorque_out_("GravityTorque_OUTPORT", false)
        , port_JointPosition_out_("JointPosition_OUTPORT", false)
    {
        addProperty("init_joint_names", init_joint_names_);
        addProperty("init_joint_positions", init_joint_positions_);
        addProperty("name", name_);
        addProperty("tool", tool_);

        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&LWRGazebo::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&LWRGazebo::gazeboUpdateHook,this,RTT::ClientThread);

        // right KUKA FRI ports
        this->ports()->addPort("JointTorqueCommand_INPORT",         port_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("KRL_CMD_INPORT",                    port_KRL_CMD_in_).doc("");
        this->ports()->addPort(port_CartesianWrench_out_);
        this->ports()->addPort(port_RobotState_out_);
        this->ports()->addPort(port_FRIState_out_);
        this->ports()->addPort(port_JointVelocity_out_);
        this->ports()->addPort(port_MassMatrix_out_);
        this->ports()->addPort(port_JointTorque_out_);
        this->ports()->addPort(port_GravityTorque_out_);
        this->ports()->addPort(port_JointPosition_out_);

        for (int i = 0; i < 7; ++i) {
            JointTorqueCommand_in_[i] = 0;
        }

        command_mode_ = false;
    }

    LWRGazebo::~LWRGazebo() {
    }

ORO_LIST_COMPONENT_TYPE(LWRGazebo)
