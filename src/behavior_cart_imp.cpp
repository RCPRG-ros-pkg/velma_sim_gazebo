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

#include "abstract_behavior.h"
#include "input_data.h"
#include "common_predicates.h"

namespace velma_core_cs_types {

class BehaviorCartImp : public BehaviorBase {
public:
    BehaviorCartImp() :
        BehaviorBase("behavior_velma_core_cs_cart_imp", "cart_imp")
    {
        addRunningComponent("CImp");
        addRunningComponent("JntLimit");
        addRunningComponent("PoseIntLeft");
        addRunningComponent("PoseIntRight");
//        addRunningComponent("FK");

        // running: [CImp, JntLimit, PoseIntLeft, PoseIntRight, lli_hi_tx, Mass]
    }

    virtual bool checkErrorCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components,
                ErrorCausePtr result) const
    {
        // check status of current component graph that makes up the transition function
        if (!allComponentsOk(components, getRunningComponents())) {
            if (result) {
                result->setBit(COMPONENT_bit, true);
            }
            return true;
        }

        // TODO: check VE state

        // TODO: check this subsystem state, eg. robot workspace, singularities

        return false;
    }

    virtual bool checkStopCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components) const
    {
        // received exactly one command for another behavior
        bool another_behavior_command = (oneCommandValid(in_data->cmd_) && !in_data->cmd_.cart_r_valid && !in_data->cmd_.cart_l_valid);
        if (another_behavior_command) {
            return true;
        }

        return false;
    }
};

};  // namespace velma_core_cs_types

REGISTER_BEHAVIOR( velma_core_cs_types::BehaviorCartImp );

