/*
 Copyright (c) 2016, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

#ifndef MANIPULATOR_MASS_MATRIX_H__
#define MANIPULATOR_MASS_MATRIX_H__

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace manipulator_mass_matrix {

class Link {
public:
    Link(const std::string &name, const gazebo::physics::LinkPtr &gz_link);
    void setInertia(double mass, const ignition::math::Vector3d &cog, double IXX, double IXY, double IXZ, double IYY, double IYZ, double IZZ);
    const std::string &getName() const;
    void setJointAcceleration(double accel);
    void setJointName(const std::string &name);
    const std::string &getJointName() const;
    void updateLocalJacobian(const ignition::math::Pose3d &childLinkToJoint, const ignition::math::Vector3d &axis);
    void setParent(std::shared_ptr<Link > &parent);
    void setChild(std::shared_ptr<Link > &child);
    void setRelativePose(const ignition::math::Pose3d &p);
    void updateMassMatrix();
    void setIndex(int index);
    int getIndex() const;
    void aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col);

    gazebo::physics::ModelPtr model_;

    gazebo::physics::LinkPtr getGazeboLink() const;

protected:
    Eigen::Matrix<double, 6, 6> mI_;
    std::string name_;
    std::string joint_name_;
    double joint_accel_;
    Eigen::Matrix<double, 6, 1> mM_dV_;
    Eigen::Matrix<double, 6, 1> mM_F_;
    Eigen::Matrix<double, 6, 1> mJacobian_;
    std::shared_ptr<Link > parent_;
    std::shared_ptr<Link > child_;
    ignition::math::Pose3d relative_pose_;
    int index_;
    gazebo::physics::LinkPtr gz_link_;    
};

class Manipulator {
public:
    Manipulator(gazebo::physics::ModelPtr model, const std::string &first_joint, const std::string &last_joint,
                            double tool_mass, const ignition::math::Vector3d &tool_cog, double tool_IXX, double tool_IXY,
                            double tool_IXZ, double tool_IYY, double tool_IYZ, double tool_IZZ);

    const Eigen::MatrixXd& getMassMatrix();
    void updatePoses(gazebo::physics::ModelPtr model);

protected:
    void setAccelerations(const Eigen::VectorXd &acc);

    typedef std::shared_ptr<Link > LinkPtr;
    typedef std::list<LinkPtr > LinkList;
    typedef std::vector<LinkPtr > LinkVec;
    LinkVec links_;
    Eigen::MatrixXd mM_;
};

}   // namespace manipulator_mass_matrix

#endif  // MANIPULATOR_MASS_MATRIX
