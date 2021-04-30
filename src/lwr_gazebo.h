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

#ifndef LWR_GAZEBO_H__
#define LWR_GAZEBO_H__

#include <std_msgs/Int32.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Inertia.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <kdl/frames.hpp>

#include "Eigen/Dense"

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>

#include "rtt_rosclock/rtt_rosclock.h"

#include <lwr_msgs/FriRobotState.h>
#include <lwr_msgs/FriIntfState.h>

#include "manipulator_mass_matrix.h"

typedef Eigen::Matrix<double, 7, 7> Matrix77d;

class LWRGazebo : public RTT::TaskContext
{
protected:
    typedef boost::array<double, 7 > Joints;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // right KUKA FRI ports
    RTT::InputPort<Joints >                 port_JointTorqueCommand_in_;  // FRIx.JointTorqueCommand
    RTT::InputPort<std_msgs::Int32 >        port_KRL_CMD_in_;             // FRIx.KRL_CMD
    RTT::OutputPort<lwr_msgs::FriRobotState >   port_RobotState_out_;     // FRIx.RobotState
    RTT::OutputPort<lwr_msgs::FriIntfState >    port_FRIState_out_;       // FRIx.FRIState
    RTT::OutputPort<Joints >                port_JointPosition_out_;      // FRIx.JointPosition
    RTT::OutputPort<Joints >                port_JointVelocity_out_;      // FRIx.JointVelocity
    RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrench_out_;    // FRIx.CartesianWrench
    RTT::OutputPort<Matrix77d >             port_MassMatrix_out_;         // FRIx.MassMatrix
    RTT::OutputPort<Joints >                port_JointTorque_out_;        // FRIx.JointTorque
    RTT::OutputPort<Joints >                port_GravityTorque_out_;      // FRIx.GravityTorque

    Joints                  JointTorqueCommand_in_;
    std_msgs::Int32         KRL_CMD_in_;
    lwr_msgs::FriRobotState RobotState_out_;
    lwr_msgs::FriIntfState  FRIState_out_;
    Joints                  JointPosition_out_;
    Joints                  JointVelocity_out_;
    geometry_msgs::Wrench   CartesianWrench_out_;
    Matrix77d               MassMatrix_out_;
    Joints                  JointTorque_out_;
    Joints                  GravityTorque_out_;

    // public methods
    LWRGazebo(std::string const& name);
    ~LWRGazebo();
    void updateHook();
    bool startHook();
    bool configureHook();
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  protected:

    // ROS parameters
    std::string name_;
    std::vector<std::string> init_joint_names_;
	std::vector<double> init_joint_positions_;
    geometry_msgs::Inertia tool_;

    Joints                  tmp_JointTorqueCommand_in_;
    std_msgs::Int32         tmp_KRL_CMD_in_;
    Joints                  tmp_JointPosition_out_;
    Joints                  tmp_JointVelocity_out_;
    geometry_msgs::Wrench   tmp_CartesianWrench_out_;
    Matrix77d               tmp_MassMatrix_out_;
    Joints                  tmp_JointTorque_out_;
    Joints                  tmp_GravityTorque_out_;

    bool data_valid_;

    bool parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c);
    void setInitialPosition(const std::vector<double > &init_q);

    gazebo::physics::ModelPtr model_;

    // torso and KUKA LWRs
    bool command_mode_;

    double tool_mass_;
    KDL::Vector tool_com_W_;

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    void getExternalForces(Joints &q);
    void getJointPositionAndVelocity(Joints &q, Joints &dq);
    void setForces(const Joints &t);
    void getGravComp(Joints &t);

    std::shared_ptr<manipulator_mass_matrix::Manipulator > mm_;

    std::vector<double > init_q_vec_;

    std::vector<std::string> link_names_;

    std::vector<gazebo::physics::JointPtr > joints_;
    std::vector<gazebo::physics::LinkPtr > links_;

    int counter_;

    ros::Time last_update_time_;
};

#endif  // LWR_GAZEBO_H__

