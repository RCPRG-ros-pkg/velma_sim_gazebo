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

#include "barrett_tactile_gazebo.h"
#include "barrett_hand_common/tactile_geometry.h"
#include "rtt_rosclock/rtt_rosclock.h"
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>

using namespace RTT;

    BarrettTactileGazebo::BarrettTactileGazebo(std::string const& name)
        : TaskContext(name, RTT::TaskContext::PreOperational)
        , median_filter_samples_(1)
        , median_filter_max_samples_(8)
        , model_(NULL)
        , data_valid_(false)
        , port_tactile_out_("BHPressureState_OUTPORT", false)
        , port_tactile_info_out_("tactile_info_OUTPORT", false)
        , port_max_pressure_out_("max_measured_pressure_OUTPORT", false)
    {
        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&BarrettTactileGazebo::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&BarrettTactileGazebo::gazeboUpdateHook,this,RTT::ClientThread);

        ts_[0] = new Tactile(median_filter_max_samples_);
        ts_[1] = new Tactile(median_filter_max_samples_);
        ts_[2] = new Tactile(median_filter_max_samples_);
        ts_[3] = new Tactile(median_filter_max_samples_);
        ts_[0]->setGeometry("finger1_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[1]->setGeometry("finger2_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[2]->setGeometry("finger3_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[3]->setGeometry("palm_info", palm_sensor_center, palm_sensor_halfside1, palm_sensor_halfside2, 0.001);

        this->ports()->addPort(port_tactile_out_);
        this->ports()->addPort("calibrate_tactile_sensors_INPORT", port_calibrate_in_);
        this->ports()->addPort("set_median_filter_INPORT", port_filter_in_);
        this->ports()->addPort(port_tactile_info_out_);
        this->ports()->addPort(port_max_pressure_out_);
        max_pressure_out_.setZero();

		this->addProperty("prefix", prefix_);

        // tactile array info
        for (int id=0; id<4; ++id) {
            pressure_info_.sensor[id].frame_id = ts_[id]->getName();
            for (int i=0; i<24; ++i)
            {
                pressure_info_.sensor[id].force_per_unit[i] = 1.0/256.0;
            }
        }

        for (int id=0; id<4; ++id)
        {
            for (int i=0; i<24; ++i)
            {
                pressure_info_.sensor[id].center[i] = ts_[id]->getCenter(i);
                pressure_info_.sensor[id].halfside1[i] = ts_[id]->getHalfside1(i);
                pressure_info_.sensor[id].halfside2[i] = ts_[id]->getHalfside2(i);
            }
        }
        port_tactile_info_out_.setDataSample(pressure_info_);
        port_tactile_out_.setDataSample(tactile_out_);
        port_max_pressure_out_.setDataSample(max_pressure_out_);
    }

    BarrettTactileGazebo::~BarrettTactileGazebo() {
    }

    bool BarrettTactileGazebo::configureHook() {
        Logger::In in("BarrettTactileGazebo::configureHook");

        if(model_.get() == NULL) {
            Logger::log() << Logger::Error << "gazebo model is NULL" << Logger::endl;
            return false;
        }

        if (prefix_.empty()) {
            Logger::log() << Logger::Error << "param 'prefix' is empty" << Logger::endl;
            return false;
        }

        std::string collision_names[4] = {prefix_ + std::string("_HandFingerOneKnuckleThreeLink_collision"),
            prefix_ + std::string("_HandFingerTwoKnuckleThreeLink_collision"),
            prefix_ + std::string("_HandFingerThreeKnuckleThreeLink_collision"),
            prefix_ + std::string("_arm_7_link_lump::") + prefix_ + std::string("_HandPalmLink_collision_1") };

        for (int i = 0; i < 4; i++) {
            for (gazebo::physics::Link_V::const_iterator it = model_->GetLinks().begin(); it != model_->GetLinks().end(); it++) {
                const std::string &link_name = (*it)->GetName();
                int col_count = (*it)->GetCollisions().size();
                bool found = false;
                for (int cidx = 0; cidx < col_count; cidx++) {
                    const std::string &col_name = (*it)->GetCollisions()[cidx]->GetName();
                    if (col_name == collision_names[i]) {
                        link_names_.push_back( link_name );
//                        vec_T_C_L_.push_back( dart_sk_->getBodyNode(link_name)->getCollisionShape(cidx)->getLocalTransform().inverse() );
                        found = true;
                        break;
                    }
                }
                if (found) {
                    break;
                }
            }
        }

        return true;
    }

    void BarrettTactileGazebo::updateHook() {
        // Synchronize with gazeboUpdate()
        RTT::os::MutexLock lock(gazebo_mutex_);
// TODO
/*
        if (!data_valid_) {
            return;
        }

        max_pressure_out_.setZero();
        for (int i=0; i<24; ++i)
        {
            tactile_out_.finger1_tip[i] = ts_[0]->getPressure(i,median_filter_samples_);
            tactile_out_.finger2_tip[i] = ts_[1]->getPressure(i,median_filter_samples_);
            tactile_out_.finger3_tip[i] = ts_[2]->getPressure(i,median_filter_samples_);
            tactile_out_.palm_tip[i] = ts_[3]->getPressure(i,median_filter_samples_);
            for (int puck_id = 0; puck_id < 4; puck_id++) {
                if (max_pressure_out_(puck_id) < ts_[puck_id]->getPressure(i,median_filter_samples_)) {
                    max_pressure_out_(puck_id) = ts_[puck_id]->getPressure(i,median_filter_samples_);
                }
            }
        }
*/
        port_max_pressure_out_.write(max_pressure_out_);
        port_tactile_out_.write(tactile_out_);
        port_tactile_info_out_.write(pressure_info_);
    }

    bool BarrettTactileGazebo::startHook() {
      return true;
    }

    bool BarrettTactileGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
        Logger::In in("BarrettTactileGazebo::gazeboConfigureHook");

        if(model.get() == NULL) {
            Logger::log() << Logger::Error << "gazebo model is NULL" << Logger::endl;
            return false;
        }

        model_ = model;

        return true;
    }

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void BarrettTactileGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    // TODO
    if (link_names_.size() != 4) {
//        std::cout << "ERROR: BarrettTactileGazebo: link_names_.size() != 4" << std::endl;
        return;
    }

    data_valid_ = true;
/*
    if (!model_dart_) {
        std::cout << "ERROR: BarrettTactileGazebo: !model_dart_" << std::endl;
        return;
    }

    RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(!trylock.isSuccessful()) {
        return;
    }

    Tactile::TactileState tact[4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 24; j++) {
            tact[i][j] = 0;
        }
    }

//    detector_->detectCollision(true, true);
    size_t collisionCount = detector_->getNumContacts();
    if (collisionCount > 0) {
        for(size_t i = 0; i < collisionCount; ++i)
        {
            const dart::collision::Contact& contact = detector_->getContact(i);
            const std::string &b1_name = contact.bodyNode1->getName();
            const std::string &b2_name = contact.bodyNode2->getName();
            double closest_dist = -1;
            double max_force = -1;
            for (int lidx = 0; lidx < 4; lidx++) {
                bool check = false;
                Eigen::Isometry3d T_W_L;
                Eigen::Vector6d ext_f;
                if (b1_name == link_names_[lidx]) {
                    check = true;
                    T_W_L = contact.bodyNode1->getTransform();
                    ext_f = contact.bodyNode1->getExternalForceLocal();
                }
                if (b2_name == link_names_[lidx]) {
                    check = true;
                    T_W_L = contact.bodyNode2->getTransform();
                    ext_f = contact.bodyNode2->getExternalForceLocal();
                }
                const Eigen::Vector3d &P_W = contact.point;
                const Eigen::Vector3d &P_C = vec_T_C_L_[lidx] * T_W_L.inverse() * P_W;
                
                if (check) {
                    for (int tidx = 0; tidx < 24; tidx++) {
                        const Eigen::Isometry3d &T_S_C = ts_[lidx]->getFrameInv(tidx);
                        const Eigen::Vector3d &P_S = T_S_C * P_C;
                        if (std::fabs(P_S.x()) < ts_[lidx]->getHalfsideLength1(tidx) && std::fabs(P_S.y()) < ts_[lidx]->getHalfsideLength2(tidx) && std::fabs(P_S.z()) < 0.005) {
                            tact[lidx][tidx] += 256;
                        }
//                        if (b1_name == link_names_[3] || b2_name == link_names_[3]) {
//                            std::cout << tidx << "  " << P_S.x() << "  " << P_S.y() << "  " << P_S.z() << std::endl;
//                        }
                    }
//                    std::cout << b1_name << " " << b2_name << "   " << ext_f.transpose() << std::endl;
                }
            }
        }
    }

    ts_[0]->updatePressure(tact[0]);
    ts_[1]->updatePressure(tact[1]);
    ts_[2]->updatePressure(tact[2]);
    ts_[3]->updatePressure(tact[3]);
*/
}

ORO_LIST_COMPONENT_TYPE(BarrettTactileGazebo)
