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
#include "barrett_hand_tactile/tactile_geometry.h"
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
        , has_optoforce_(false)
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

        std::string collision_names[4] = {
            prefix_ + std::string("_HandFingerOneKnuckleThreeLink_collision"),
            prefix_ + std::string("_HandFingerTwoKnuckleThreeLink_collision"),
            prefix_ + std::string("_HandFingerThreeKnuckleThreeLink_collision"),
            prefix_ + std::string("_arm_7_link_lump::") + prefix_ + std::string("_HandPalmLink_collision_1") };

        const std::string optoforce_link_name_example = prefix_ +
                                        std::string("_HandFingerOneKnuckleThreeOptoforceSensor");
        for (int i = 0; i < 4; i++) {
            for (gazebo::physics::Link_V::const_iterator it = model_->GetLinks().begin();
                                                            it != model_->GetLinks().end(); it++) {

                const std::string &link_name = (*it)->GetName();

                if ( link_name == (optoforce_link_name_example) ) {
                    has_optoforce_ = true;
                }

                int col_count = (*it)->GetCollisions().size();
                bool found = false;
                for (int cidx = 0; cidx < col_count; cidx++) {
                    const std::string &col_name = (*it)->GetCollisions()[cidx]->GetName();
                    if (col_name == collision_names[i]) {
                        link_names_.push_back( link_name );
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
        
        if (has_optoforce_) {
            // nothing to do - there are no tactile sensors (except palm)
            return;
        }
        // TODO
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
    if (has_optoforce_) {
        // nothing to do - there are no tactile sensors (except palm)
        return;
    }
    // TODO
    if (link_names_.size() != 4) {
        return;
    }

    data_valid_ = true;
}

ORO_LIST_COMPONENT_TYPE(BarrettTactileGazebo)
