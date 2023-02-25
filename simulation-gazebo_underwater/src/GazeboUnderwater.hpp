#ifndef _GAZEBOUNDERWATER_HPP_
#define _GAZEBOUNDERWATER_HPP_

#include "Gazebo7Shims.hpp"
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "DataTypes.hpp"

namespace gazebo_underwater
{
    class GazeboUnderwater : public gazebo::ModelPlugin
    {
            typedef gazebo::physics::ModelPtr ModelPtr;
            typedef gazebo::physics::WorldPtr WorldPtr;
            typedef gazebo::physics::LinkPtr LinkPtr;
            typedef gazebo::physics::Inertial Inertial;
            typedef gazebo::msgs::Pose PoseMSG;
            typedef gazebo_underwater::msgs::CompensatedMass CompMassMSG;

        private:
            void updateBegin(gazebo::common::UpdateInfo const& info);
            void applyBuoyancy();
            void applyDamp();
            void applyCoriolisAddedInertia();
            void compensateGzEffort();
            LinkPtr getReferenceLink(ModelPtr model, sdf::ElementPtr sdf) const;
            void loadParameters();
            template <typename T>
            T getParameter(std::string parameter_name, std::string dimension, T default_value) const;
            double calculateSubmersedRatio() const;
            Inertial computeModelInertial(ModelPtr model) const;
            Vector6 getModelFrameVelocities();
            void publishInertia(Matrix6 const& comp_inertia, ignition::math::Vector3d const& cog);
            void initComNode(void);
            void readFluidVelocity(const ConstVector3dPtr&);

            std::vector<Matrix6> convertToMatrices(const std::string &matrices);
            Matrix6 convertToMatrix(const std::string &matrix);

            ModelPtr model;
            WorldPtr world;
            LinkPtr link;
            gazebo::transport::NodePtr node;
            gazebo::transport::PublisherPtr compensatedMassPublisher;
            gazebo::transport::SubscriberPtr fluidVelocitySubscriber;
            Inertial modelInertial;

            sdf::ElementPtr sdf;
            std::vector<gazebo::event::ConnectionPtr> eventHandler;

            // Inertia of Gazebo's model (M)
            Matrix6 gzInertia;
            // Added intertia of a rigid body robot (Ma)
            Matrix6 addedInertia;
            // M*(M+Ma)⁻¹
            Matrix6 compInertia;
            // M*(M+Ma)⁻¹ - I
            Matrix6 compInertiaEye;

            // Matrices of dampings.
            // If vector has two elements, they will be the linear and quadratic
            // dampin respectivly.
            // If vector has 6 elements they will be quadratic damping of
            // linear velocities x,y,z followed by the angular velocities x,y,z
            std::vector<Matrix6> dampingCoefficients;

            ignition::math::Vector3d centerOfBuoyancy;
            ignition::math::Vector3d fluidVelocity;

            double waterLevel;       // dimension in meter
            double buoyancy;

        public:
            GazeboUnderwater();
            ~GazeboUnderwater();
            virtual void Load(ModelPtr _parent, sdf::ElementPtr _sdf);
    };

    GZ_REGISTER_MODEL_PLUGIN(GazeboUnderwater)
} // end namespace gazebo_underwater

#endif
