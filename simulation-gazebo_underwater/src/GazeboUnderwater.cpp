#include "GazeboUnderwater.hpp"
#include <gazebo/common/Exception.hh>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/regex.hpp>
#include <stdlib.h>

using namespace std;
using namespace gazebo;
using namespace ignition::math;

namespace gazebo_underwater
{
    GazeboUnderwater::GazeboUnderwater()
    {
    }

    GazeboUnderwater::~GazeboUnderwater()
    {
        node->Fini();
    }

    void GazeboUnderwater::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        gzmsg << "GazeboUnderwater: Loading underwater environment." << endl;

        sdf = _sdf;

        model = _model;
        world = _model->GetWorld();
        link  = getReferenceLink(model, _sdf);
        initComNode();
        loadParameters();

        // Each simulation step the Update method is called to update the simulated sensors and actuators
        eventHandler.push_back(
                event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&GazeboUnderwater::updateBegin,this, _1)));
    }

    template <class T>
    T GazeboUnderwater::getParameter(string parameter_name, string dimension, T default_value) const
    {
        T var = default_value;
        if(sdf->HasElement(parameter_name.c_str()))
        {
            var = sdf->Get< T >(parameter_name.c_str());
            gzmsg << "GazeboUnderwater: " + parameter_name + ": (" << var << ") "
                    + dimension  << endl;
        }else{
            gzmsg << "GazeboUnderwater: " + parameter_name + ": using default ("
                    << default_value << ") " + dimension << endl;
        }
        return var;
    }

    physics::LinkPtr GazeboUnderwater::getReferenceLink(physics::ModelPtr model, sdf::ElementPtr sdf) const
    {
        if(sdf->HasElement("link_name"))
        {
            physics::LinkPtr link = model->GetLink( sdf->Get<string>("link_name") );
            gzmsg << "GazeboUnderwater: reference link: " << link->GetName() << endl;
            if (!link) {
                string msg = "GazeboUnderwater: link " + sdf->Get<string>("link_name")
                        + " not found in model " + model->GetName();
                gzthrow(msg);
            }
            return link;
        }else if (model->GetLinks().empty()) {
            gzthrow("GazeboUnderwater: no link defined in model");
        }else{
            physics::LinkPtr link = model->GetLinks().front();
            gzmsg << "GazeboUnderwater: reference link not defined, using instead: " << link->GetName() << endl;
            return link;
        }
    }

    physics::Inertial GazeboUnderwater::computeModelInertial(physics::ModelPtr model) const
    {
        Inertial inertial(0);
        inertial.SetMOI(GzMatrix3(Matrix3d::Zero));
        physics::Link_V links = model->GetLinks();
        for (physics::Link_V::iterator it = links.begin(); it != links.end(); ++it)
            if(!(*it)->GetKinematic())
            {
                // Set Inertial's CoG related with the parent link
                Inertial temp = *(*it)->GetInertial();
                Pose3d pose = GzGetIgn((**it), RelativePose, ());
                pose.Pos() += pose.Rot().RotateVector(GzGetIgn(temp, CoG, ()));
                temp.SetCoG(pose);
                inertial += temp;
            }
        return inertial;
    }

    void GazeboUnderwater::loadParameters(void)
    {
        waterLevel = getParameter<double>("water_level","meters", 0.0);
        fluidVelocity = getParameter<Vector3d>("fluid_velocity","m/s",Vector3d(0,0,0));
        modelInertial = computeModelInertial(model);

        // buoyancy must be the difference between the buoyancy when the model is completely submersed and the model weight
        buoyancy = getParameter<double>("buoyancy","N", 5);
        buoyancy = abs(buoyancy) + GzGet(modelInertial, Mass, ()) * world->Gravity().Length();
        // centerOfBuoyancy must be positioned related to the model's center of gravity.
        centerOfBuoyancy = getParameter<Vector3d>("center_of_buoyancy","meters",
                Vector3d(0, 0, 0.15));
        centerOfBuoyancy += GzGetIgn(modelInertial, CoG, ());
        std::string damp_matrices;
        damp_matrices += "[50 0 0 0 0 0;";
        damp_matrices += "0 50 0 0 0 0;";
        damp_matrices += "0 0 50 0 0 0;";
        damp_matrices += "0 0 0 45 0 0;";
        damp_matrices += "0 0 0 0 45 0;";
        damp_matrices += "0 0 0 0 0 45]";
        damp_matrices += "[40 0 0 0 0 0;";
        damp_matrices += "0 40 0 0 0 0;";
        damp_matrices += "0 0 40 0 0 0;";
        damp_matrices += "0 0 0 35 0 0;";
        damp_matrices += "0 0 0 0 35 0;";
        damp_matrices += "0 0 0 0 0 35]";
        dampingCoefficients = convertToMatrices(getParameter<string>("damping_coefficients","N/vel^n / vel^n={m/s, rad/s, m2/s2. rad2/s2}",damp_matrices));

        std::string extra_inertia;
        extra_inertia += "[0 0 0 0 0 0;";
        extra_inertia += "0 0 0 0 0 0;";
        extra_inertia += "0 0 0 0 0 0;";
        extra_inertia += "0 0 0 0 0 0;";
        extra_inertia += "0 0 0 0 0 0;";
        extra_inertia += "0 0 0 0 0 0]";
        addedInertia = convertToMatrix(getParameter<string>("added_inertia","Kg, Kg.m2", extra_inertia));

        gzInertia.top_left = GzGet(modelInertial, Mass, ()) * Matrix3d::Identity;
        gzInertia.bottom_right = IgnMatrix3(GzGet(modelInertial, MOI, ()));
        Matrix6 sum_inertia = gzInertia + addedInertia;
        gzmsg << "GazeboUnderwater: Inertia (Model_Inertia + Added_Inertia)" << endl;
        gzmsg << "Inertia top_left:     " << endl << sum_inertia.top_left;
        gzmsg << "Inertia top_right:    " << endl << sum_inertia.top_right;
        gzmsg << "Inertia bottom_left:  " << endl << sum_inertia.bottom_left;
        gzmsg << "Inertia bottom_right: " << endl << sum_inertia.bottom_right << endl;

        for (unsigned int i=0; i<dampingCoefficients.size(); i++)
        {   gzmsg <<"GazeboUnderwater: Damping["<<i<<"]" <<std::endl;
            gzmsg <<"Damping["<<i<<"] top_left:     " << endl << dampingCoefficients[i].top_left;
            gzmsg <<"Damping["<<i<<"] top_right:    " << endl << dampingCoefficients[i].top_right;
            gzmsg <<"Damping["<<i<<"] bottom_left:  " << endl << dampingCoefficients[i].bottom_left;
            gzmsg <<"Damping["<<i<<"] bottom_right: " << endl << dampingCoefficients[i].bottom_right << endl;
        }

        gzmsg << "GazeboUnderwater: Model's weight: "   << GzGet(modelInertial, Mass, ()) * model->GetWorld()->Gravity().Length() << " N" << endl;
        gzmsg << "GazeboUnderwater: Model's CoG: ("     << GzGetIgn(modelInertial, CoG, ()) << ") meters" << endl;
        gzmsg << "GazeboUnderwater: Model's buoyancy: " << buoyancy << " N" << endl;
        gzmsg << "GazeboUnderwater: Model's CoB: ("     << centerOfBuoyancy << ") meters "<< endl;

        // gzInertia is symmetric positive definite (SPD) matrix.
        // addedInertia should be symmetric positive semidefinite
        // sum_inertia is positive definite so it has inverse.
        compInertia = gzInertia * sum_inertia.Inverse();
        compInertiaEye = compInertia - Matrix6::Identity();
    }

    void GazeboUnderwater::updateBegin(common::UpdateInfo const& info)
    {
        publishInertia(compInertia, GzGetIgn(modelInertial, CoG, ()));
        applyBuoyancy();
        applyDamp();
        applyCoriolisAddedInertia();
        compensateGzEffort();
    }

    void GazeboUnderwater::applyDamp()
    {
        Vector6 vel = getModelFrameVelocities();

        Vector6 damp;
        if(dampingCoefficients.size() == 2)
        {
            Vector6 vel_square(vel.top*vel.top.Abs(), vel.bottom*vel.bottom.Abs());
            damp = dampingCoefficients[0] * vel + dampingCoefficients[1] * vel_square;
        }
        else if(dampingCoefficients.size() == 6)
        {
            Matrix6 dampMatrix;
            Vector6 vel_abs(vel.top.Abs(), vel.bottom.Abs());
            for(size_t i=0; i < 3; i++)
            {
                dampMatrix += dampingCoefficients[i] * vel_abs.top[i];
                dampMatrix += dampingCoefficients[i+3] * vel_abs.bottom[i];
            }
            damp = dampMatrix * vel;
        }
        else
            gzthrow("GazeboUnderwater: Damping Parameter has wrong dimension!");

        damp = compInertia * damp * -1.0;

        link->AddLinkForce(damp.top, GzGetIgn(modelInertial, CoG, ()));
        link->AddRelativeTorque(damp.bottom);
    }

    void GazeboUnderwater::applyBuoyancy()
    {
        double submersedRatio = calculateSubmersedRatio();
        Vector3d modelBuoyancy = Vector3d(0, 0, submersedRatio * buoyancy);
        Vector6 effort;
        effort.top = GzGetIgn((*link), WorldPose, ()).Rot().RotateVectorReverse(modelBuoyancy);
        effort.bottom = (centerOfBuoyancy - GzGetIgn(modelInertial, CoG, ())).Cross(effort.top);

        effort = compInertia * effort;

        link->AddLinkForce(effort.top, GzGetIgn(modelInertial, CoG, ()));
        link->AddRelativeTorque(effort.bottom);
    }

    double GazeboUnderwater::calculateSubmersedRatio() const
    {
        Box linkBoundingBox = GzGetIgn((*link), BoundingBox, ());
        // Distance of the lower part of the bounding box to the surface
        // It is positive when submerged
        double distanceToSurface = waterLevel - linkBoundingBox.Min().Z();
        double submersedRatio = distanceToSurface / linkBoundingBox.ZLength();
        
        if (submersedRatio < 0)
            return 0;
        else if (submersedRatio > 1)
            return 1;
        else return submersedRatio;
    }

    void GazeboUnderwater::applyCoriolisAddedInertia()
    {
        /**
         * Based on McFarland[2013] and Fossen[1994]
         * coriolisEffect = H(M*v)*v
         * M = inertiaMatrix; v = velocity
         * Operator H: R^6 -> R^(6x6).
         *      H(v) = [0(3x3), J(v.head(3));
         *              J(v.head(3)),  J(v.tail(3))]
         * Operator J: R^3 -> R^(3x3) (the so(3) operator, skew-symmetric matrix)
         *      J([v1; v2; v3]) = [ 0 ,-v3, v2;
         *                          v3, 0 ,-v1;
         *                         -v2, v1, 0]
         * Cross product:
         *      J(v.head(3)) * v.tail(3) = v.head(3) X v.tail(3)
         */
        Vector6 velocities = getModelFrameVelocities();
        Vector6 momentum = addedInertia * velocities;
        Vector6 coriolisEffect( momentum.top.Cross(velocities.bottom),
                    momentum.top.Cross(velocities.top) + momentum.bottom.Cross(velocities.bottom));

        coriolisEffect = compInertia * coriolisEffect;

        link->AddLinkForce(coriolisEffect.top, GzGetIgn(modelInertial, CoG, ()));
        link->AddRelativeTorque(coriolisEffect.bottom);
    }

    void GazeboUnderwater::compensateGzEffort()
    {
        /**
         * --AUV acceleration--
         * acceleration = (M+Ma)^-1 *
         * (Thruster + Coriolis + Coriolis_added_mass + Gravity_effect - Damping)
         * acceleration = (M+Ma)^-1 * F;
         * F =
         * (Thruster + Coriolis + Coriolis_added_mass + Gravity_effect - Damping)
         *
         * --Acceleration computed by Gazebo--
         *  acceleration' = M^-1 * F'
         *
         * -- Make accelerations equal--
         * acceleration = acceleration'
         * (M+Ma)^-1 * F = M^-1 * F'
         * F' = F + C  => (M+Ma)^-1 * F = M^-1 * (F + C)
         * C = (M*(M+Ma)^-1 - I) * F;
         */

        // Consider Coriolis of model's inertia
        Vector6 velocities = getModelFrameVelocities();
        Vector6 momentum = gzInertia * velocities;
        Vector6 effort( momentum.top.Cross(velocities.bottom),
                    momentum.top.Cross(velocities.top) + momentum.bottom.Cross(velocities.bottom));

        // Consider gravity
        Vector3d weight( GzGet(modelInertial, Mass, ()) * world->Gravity() );
        effort.top += GzGetIgn((*link), WorldPose, ()).Rot().RotateVectorReverse( weight );

        effort = compInertiaEye * effort;

        link->AddLinkForce(effort.top, GzGetIgn(modelInertial, CoG, ()));
        link->AddRelativeTorque(effort.bottom);
    }

    std::vector<Matrix6> GazeboUnderwater::convertToMatrices(const std::string &matrices)
    {
        std::vector<Matrix6> ret;
        size_t start = matrices.find("[");
        while(start != string::npos)
        {
            size_t end = matrices.find("]", start);
            ret.push_back(convertToMatrix(matrices.substr(start, end+1)));
            start = matrices.find("[", end + 1);
        }
        if (ret.size() != 2 && ret.size() != 6)
            gzthrow("GazeboUnderwater: Damping Parameters has not 2 or 6 matrices!");
        return ret;
    }

    Matrix6 GazeboUnderwater::convertToMatrix(const std::string &matrix)
    {
        Matrix6 ret;
        std::vector<std::string> splitted;
        if(matrix.compare(0,1,"[") || matrix.compare((matrix.size()-1),1,"]"))
            gzthrow("GazeboUnderwater: Matrix is not delimetd by \"[ ]\"!");
        std::string numeric_matrix = matrix.substr(1,matrix.size()-2);
        boost::split( splitted, numeric_matrix, boost::is_any_of( ";" ), boost::token_compress_on );
        if (splitted.size() != 6)
            gzthrow("GazeboUnderwater: Matrix has not 6 lines!");
        for( size_t i=0; i<6; i++)
        {
            std::vector<std::string> line;
            boost::trim(splitted[i]);
            boost::split( line, splitted[i], boost::is_any_of( " " ), boost::token_compress_on );
            if (line.size() != 6)
                gzthrow("GazeboUnderwater: Line has not 6 columns!" + splitted.size());
            for(size_t j=0; j<6; j++)
            {
                if(i<3 && j<3)
                    ret.top_left(i,j) = atof(line.at(j).c_str());
                else if(i<3 && j>=3)
                    ret.top_right(i,j-3) = atof(line.at(j).c_str());
                else if(i>=3 && j<3)
                    ret.bottom_left(i-3,j) = atof(line.at(j).c_str());
                else if(i>=3 && j>=3)
                    ret.bottom_right(i-3,j-3) = atof(line.at(j).c_str());
            }
        }
        return ret;
    }

    Vector6 GazeboUnderwater::getModelFrameVelocities()
    {
        Vector6 velocities;
        // Calculates the difference between the model and fluid velocity relative to the world frame
        Vector3d CoG = GzGetIgn(modelInertial, CoG, ());
        Vector3d velocityDifference = GzGetIgn((*link), WorldLinearVel, (CoG)) - fluidVelocity;
        velocities.top = GzGetIgn((*link), WorldPose, ()).Rot().RotateVectorReverse( velocityDifference );
        velocities.bottom = GzGetIgn((*link), RelativeAngularVel, ());
        return velocities;
    }

    void GazeboUnderwater::publishInertia(Matrix6 const& comp_inertia, Vector3d const& cog)
    {
        CompMassMSG compMassMSG;
        compMassMSG.mutable_matrix()->CopyFrom(comp_inertia.ConvertToMsg());
        compMassMSG.mutable_cog()->CopyFrom(gazebo::msgs::Convert(cog));
        if(compensatedMassPublisher->HasConnections())
            compensatedMassPublisher->Publish(compMassMSG);
    }

    void GazeboUnderwater::initComNode(void)
    {
        // Initialize communication node and subscribe to gazebo topic
        node = transport::NodePtr(new transport::Node());
        node->Init();

        string topicName = model->GetName() + "/compensated_mass";
        compensatedMassPublisher = node->Advertise<CompMassMSG>("~/" + topicName);
        gzmsg <<"GazeboUnderwater: create gazebo topic /gazebo/"+ GzGet((*model->GetWorld()), Name, ())
            + "/" + topicName << endl;

        topicName = model->GetName() + "/fluid_velocity";
        fluidVelocitySubscriber = node->Subscribe(
            "~/" + topicName, &GazeboUnderwater::readFluidVelocity, this, true);
        gzmsg <<"GazeboUnderwater: created gazebo topic /gazebo/"+ GzGet((*model->GetWorld()), Name, ())
            + "/" + topicName << endl;
    }

    void GazeboUnderwater::readFluidVelocity(const ConstVector3dPtr& velocity) {
        fluidVelocity = Vector3d(velocity->x(), velocity->y(), velocity->z());
    }
}
