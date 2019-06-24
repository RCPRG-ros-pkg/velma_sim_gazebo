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

#include "barrett_hand_gazebo.h"
#include <rtt/Component.hpp>

    BarrettHandGazebo::BarrettHandGazebo(std::string const& name)
        : TaskContext(name, RTT::TaskContext::PreOperational)
        , too_big_force_counter_(3, 0)
        , data_valid_(false)
//        , port_q_out_("q_OUTPORT", false)
//        , port_t_out_("t_OUTPORT", false)
//        , port_status_out_("status_OUTPORT", false)
        , disable_component_(false)
        , can_id_base_(-1)
        , first_step_(true)
    {
        addProperty("prefix", prefix_);
        addProperty("disable_component", disable_component_);
        addProperty("can_id_base", can_id_base_);

        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&BarrettHandGazebo::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&BarrettHandGazebo::gazeboUpdateHook,this,RTT::ClientThread);

        // right hand ports
//        this->ports()->addPort("q_INPORT",      port_q_in_);
//        this->ports()->addPort("v_INPORT",      port_v_in_);
//        this->ports()->addPort("t_INPORT",      port_t_in_);
//        this->ports()->addPort("mp_INPORT",     port_mp_in_);
//        this->ports()->addPort("hold_INPORT",   port_hold_in_);
//        this->ports()->addPort(port_q_out_);
//        this->ports()->addPort(port_t_out_);
//        this->ports()->addPort(port_status_out_);
        //this->ports()->addPort("BHTemp",        port_temp_out_);
//        this->ports()->addPort("max_measured_pressure_INPORT", port_max_measured_pressure_in_);
//        this->ports()->addPort("reset_fingers_INPORT", port_reset_in_);
//        q_in_.setZero();
//        v_in_.setZero();
//        t_in_.setZero();
        mp_in_ = 0.0;
        hold_in_ = 0;    // false
        max_measured_pressure_in_.setZero();
        //temp_out_.temp.resize(8);
        //port_temp_out_.setDataSample(temp_out_);
//        status_out_ = STATUS_IDLE1 | STATUS_IDLE2 | STATUS_IDLE3 | STATUS_IDLE4;
        for (int i = 0; i < 4; ++i) {
//            status_idle_[i] = true;
            status_overcurrent_[i] = false;
//            move_hand_[i] = false;
        }
        clutch_break_[0] = clutch_break_[1] = clutch_break_[2] = false;
    }

    BarrettHandGazebo::~BarrettHandGazebo() {
    }

ORO_LIST_COMPONENT_TYPE(BarrettHandGazebo)

