#ifndef _GAZEBOTHRUSTER_HPP_
#define _GAZEBOTHRUSTER_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_underwater/DataTypes.hpp>

#include "msgs.pb.h"

namespace gazebo_thruster
{
    class GazeboThruster : public gazebo::ModelPlugin
    {
    public:
        GazeboThruster();
        ~GazeboThruster();
        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

        typedef const boost::shared_ptr<const gazebo_thruster::msgs::Thrusters> ThrustersMSG;
        typedef const boost::shared_ptr<const gazebo_underwater::msgs::CompensatedMass> CompMassMSG;

        void readInput(ThrustersMSG const& thrustersMSG);
        void readCompensatedMass(CompMassMSG const& compMassMSG);

        struct Thruster{
            std::string name;
            double minThrust;
            double maxThrust;
            double effort;
            ignition::math::Vector3d added_mass_compensated_direction;
            ignition::math::Vector3d added_mass_compensated_position;
        };
        
        gazebo_underwater::Matrix6 mass_matrix;
    private:
        void updateBegin(gazebo::common::UpdateInfo const& info);
        std::vector<Thruster> loadThrusters();
        void checkThrusters( std::vector<Thruster> );
        void initComNode();
        void checkThrustLimits(std::vector<Thruster>::iterator thruster);
        double updateEffort(gazebo_thruster::msgs::Thruster thrusterCMD);
        void updateCompensatedEffort(gazebo_underwater::Matrix6 const& matrix, ignition::math::Vector3d const& cog);

        std::vector<gazebo::event::ConnectionPtr> eventHandler;
        gazebo::transport::NodePtr node;
        gazebo::transport::SubscriberPtr thrusterSubscriber;
        gazebo::transport::SubscriberPtr compensatedMassSubscriber;
        gazebo::physics::ModelPtr model;

        template <typename T>
        T getParameter(sdf::ElementPtr thrusterElement, std::string parameter_name,
                std::string dimension, T default_value);
        std::vector<Thruster> thrusters;
    };
    GZ_REGISTER_MODEL_PLUGIN(GazeboThruster)
} 

#endif
