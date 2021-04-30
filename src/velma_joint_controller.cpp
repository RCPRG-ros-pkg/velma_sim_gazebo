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

#include "velma_joint_controller.h"

VelmaJointController::VelmaJointController(gazebo::physics::JointPtr joint, double period,
                                        double Kp, double Ki,
                                        double i_min, double i_max, double u_min, double u_max)
: joint_(joint)
, period_(period)
, Kp_(Kp)
, Ki_(Ki)
, u_max_(u_max)
, u_min_(u_min)
, i_max_(i_max)
, i_min_(i_min)
, e_sum_(0.0)
, u_(0.0)
, q_dst_(0)
, is_initialized_(false)
{}

void VelmaJointController::setParameters(double Kp, double Ki, double i_min, double i_max,
                                                                    double u_min, double u_max) {
    Kp_ = Kp;
    Ki_ = Ki;
    u_max_ = u_max;
    u_min_ = u_min;
    i_max_ = i_max;
    i_min_ = i_min;
}

void VelmaJointController::setTargetPosition(double q_dst) {
    q_dst_ = q_dst;
}

void VelmaJointController::update() {
    double q_msr = joint_->Position();
    double e = q_dst_ - q_msr;
    e_sum_ += e * period_;

    if (e_sum_ > i_max_) {
        e_sum_ = i_max_;
    }

    if (e_sum_ < i_min_) {
        e_sum_ = i_min_;
    }

    u_ = Kp_ * e + Ki_ * e_sum_;
    if (u_ > u_max_) {
        u_ = u_max_;
    }
    if (u_ < u_min_) {
        u_ = u_min_;
    }
    joint_->SetForce(0, u_);
}

double VelmaJointController::getLastCommand() const {
    return u_;
}
