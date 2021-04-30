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
#include <rtt/Logger.hpp>
#include "barrett_hand_controller/BarrettHandCan.h"

using namespace RTT;

void BarrettHandGazebo::updateHook() {
//    Logger::In in(std::string("BarrettHandGazebo::updateHook ") + getName());
    Logger::In in(getName());

    // Synchronize with gazeboUpdate()
    RTT::os::MutexLock lock(gazebo_mutex_);

    if (!data_valid_) {
        //Logger::In in("BarrettHandGazebo::updateHook");
        //Logger::log() << Logger::Debug << "gazebo is not initialized" << Logger::endl;
        return;
    }
    else {
        //Logger::log() << Logger::Debug << Logger::endl;
    }

//    if (getName() == "RightHand") {
//            Logger::log() << Logger::Info << "a " << q_out_.transpose() << Logger::endl;
//            Eigen::Matrix<int, 4, 1 > status_idle;
//            for (int i=0; i<4; ++i) {
//                status_idle(i) = status_idle_[i];
//            }
//            Logger::log() << Logger::Info << status_idle.transpose() << Logger::endl;
//            Logger::log() << Logger::Info << "a " << q_out_.transpose() << Logger::endl;
//    }
    
    //
    // BarrettHand
    //
    hw_can_.jp_[0] = q_out_(1)*50.0*4096.0/2.0/M_PI;
    hw_can_.p_[0] = (q_out_(2) + q_out_(1)) * 4096.0/(1.0/125.0 + 1.0/375.0)/2.0/M_PI;
    hw_can_.jp_[1] = q_out_(4)*4096.0*50.0/2.0/M_PI;
    hw_can_.p_[1] = (q_out_(5) + q_out_(4))*4096.0/(1.0/125.0 + 1.0/375.0)/2.0/M_PI;
    hw_can_.jp_[2] = q_out_(6)*4096.0*50.0/2.0/M_PI;
    hw_can_.p_[2] = (q_out_(7) + q_out_(6))*4096.0/(1.0/125.0 + 1.0/375.0)/2.0/M_PI;
    hw_can_.p_[3] = q_out_(0)*35840.0/M_PI;

    hw_can_.processPuckMsgs();

    port_js_out_.write(js_out_);
}

bool BarrettHandGazebo::startHook() {
  return true;
}

bool BarrettHandGazebo::configureHook() {
    Logger::In in("BarrettHandGazebo::configureHook");
    if (prefix_.empty()) {
        Logger::log() << Logger::Error << "param 'prefix' is empty" << Logger::endl;
        return false;
    }

    if (can_id_base_ < 0) {
        Logger::log() << Logger::Error << "param 'can_id_base' is not set" << Logger::endl;
        return false;
    }

    hw_can_.configure(this, can_id_base_);

    std::string hand_joint_names[] = {
        "_HandFingerOneKnuckleOneJoint",
        "_HandFingerOneKnuckleTwoJoint",
        "_HandFingerOneKnuckleThreeJoint",
        "_HandFingerTwoKnuckleOneJoint",
        "_HandFingerTwoKnuckleTwoJoint",
        "_HandFingerTwoKnuckleThreeJoint",
        "_HandFingerThreeKnuckleTwoJoint",
        "_HandFingerThreeKnuckleThreeJoint" };

    double pid_params[8][6] = {
        {sp_kp_, sp_ki_, sp_min_i_, sp_max_i_, sp_min_cmd_, sp_max_cmd_},
        {k2_kp_, k2_ki_, k2_min_i_, k2_max_i_, k2_min_cmd_, k2_max_cmd_},
        {k3_kp_, k3_ki_, k3_min_i_, k3_max_i_, k3_min_cmd_, k3_max_cmd_},
        {sp_kp_, sp_ki_, sp_min_i_, sp_max_i_, sp_min_cmd_, sp_max_cmd_},
        {k2_kp_, k2_ki_, k2_min_i_, k2_max_i_, k2_min_cmd_, k2_max_cmd_},
        {k3_kp_, k3_ki_, k3_min_i_, k3_max_i_, k3_min_cmd_, k3_max_cmd_},
        {k2_kp_, k2_ki_, k2_min_i_, k2_max_i_, k2_min_cmd_, k2_max_cmd_},
        {k3_kp_, k3_ki_, k3_min_i_, k3_max_i_, k3_min_cmd_, k3_max_cmd_},
    };
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 6; j++) {
            prev_pid_params_[i][j] = pid_params[i][j];
        }
    }

    for (int i = 0; i < 8; i++) {
        std::string name( prefix_ + hand_joint_names[i] );
        gazebo::physics::JointPtr joint = model_->GetJoint(name);
        joints_.push_back(joint);
        //joint_scoped_names_.push_back(joint->GetScopedName());
        joint->SetEffortLimit(0, 1);

        vjc_.push_back( VelmaJointController(joint, 0.002, pid_params[i][0], pid_params[i][1],
            pid_params[i][2], pid_params[i][3], pid_params[i][4], pid_params[i][5]) );

        js_out_.name.push_back(name);
        js_out_.position.push_back(0.0);
        js_out_.velocity.push_back(0.0);
        js_out_.effort.push_back(0.0);
    }

    for (int i = 0; i < 3; i++) {
        clutch_break_[i] = false;
    }

    RTT::Service::shared_ptr tc_rosparam = this->provides("rosparam");

    tc_rosparam_getAll = tc_rosparam->getOperation("getAll");
    if (!tc_rosparam_getAll.ready()) {
        Logger::log() << Logger::Error
        << "could not get ROS parameter getAll operation" << Logger::endl;
    }


    

    return true;
}

