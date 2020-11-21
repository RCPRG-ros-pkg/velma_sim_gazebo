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

#ifndef BARRETT_HAND_GAZEBO_H__
#define BARRETT_HAND_GAZEBO_H__

#include <std_msgs/Empty.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "Eigen/Dense"

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>

#include "rtt_rosclock/rtt_rosclock.h"

#include <barrett_hand_hw_sim/barrett_hand_hw_can.h>

class BarrettHandGazebo : public RTT::TaskContext
{
protected:
    typedef Eigen::Matrix<double, 4, 1> Dofs;
    typedef Eigen::Matrix<double, 8, 1> Joints;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double          mp_in_;
    int32_t         hold_in_;
    Dofs max_measured_pressure_in_;
    Joints q_out_;
    Joints t_out_;
    //barrett_hand_msgs::BHTemp temp_out_;

    // public methods
    BarrettHandGazebo(std::string const& name);
    ~BarrettHandGazebo();
    void updateHook();
    bool startHook();
    bool configureHook();
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  protected:

    enum {STATUS_OVERCURRENT1 = 0x0001, STATUS_OVERCURRENT2 = 0x0002, STATUS_OVERCURRENT3 = 0x0004, STATUS_OVERCURRENT4 = 0x0008,
        STATUS_OVERPRESSURE1 = 0x0010, STATUS_OVERPRESSURE2 = 0x0020, STATUS_OVERPRESSURE3 = 0x0040,
        STATUS_TORQUESWITCH1 = 0x0100, STATUS_TORQUESWITCH2 = 0x0200, STATUS_TORQUESWITCH3 = 0x0400,
        STATUS_IDLE1 = 0x1000, STATUS_IDLE2 = 0x2000, STATUS_IDLE3 = 0x4000, STATUS_IDLE4 = 0x8000 };

    double clip(double n, double lower, double upper) const;
    double getFingerAngle(unsigned int fidx) const;

    // parameters
    std::string prefix_;
    int can_id_base_;
    double sp_kp_;
    double sp_ki_;
    double sp_kd_;
    double sp_min_i_;
    double sp_max_i_;
    double sp_min_cmd_;
    double sp_max_cmd_;
    double k2_kp_;
    double k2_ki_;
    double k2_kd_;
    double k2_min_i_;
    double k2_max_i_;
    double k2_min_cmd_;
    double k2_max_cmd_;
    double k3_kp_;
    double k3_ki_;
    double k3_kd_;
    double k3_min_i_;
    double k3_max_i_;
    double k3_min_cmd_;
    double k3_max_cmd_;

    gazebo::physics::ModelPtr model_;
    bool data_valid_;

    // BarrettHand
    std::vector<gazebo::physics::JointPtr> joints_;
    std::vector<std::string> joint_scoped_names_;

    std::vector<int > too_big_force_counter_;
    bool status_overcurrent_[4];

    bool clutch_break_[3];
    double clutch_break_angle_[3];

    double finger_int_[4];

    gazebo::physics::JointController *jc_;

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    bool disable_component_;

    BarrettHandHwCAN hw_can_;

    ros::Time last_update_time_;

    bool first_step_;
};

#endif  // BARRETT_HAND_GAZEBO_H__

