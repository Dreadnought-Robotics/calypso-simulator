#include "GazeboThruster.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_thruster;
using namespace ignition::math;

GazeboThruster::GazeboThruster()
{
}


GazeboThruster::~GazeboThruster()
{
    node->Fini();
}


void GazeboThruster::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{
    model = _model;
    gzmsg << "GazeboThruster: loading thrusters from model: " << model->GetName() << endl;

    std::vector<Thruster> thrusters = loadThrusters();
    checkThrusters(thrusters);
    this->thrusters = thrusters;

    initComNode();

    eventHandler.push_back(
            event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboThruster::updateBegin,this, _1)));
}


template <class T>
T GazeboThruster::getParameter(sdf::ElementPtr thrusterElement,
        string parameter_name, string dimension, T default_value)
{
    T var = default_value;
    if(thrusterElement->HasElement(parameter_name.c_str()))
    {
        var = thrusterElement->Get< T >(parameter_name.c_str());
        gzmsg << "GazeboThruster: " + parameter_name + ": " << var << " " +
                dimension  << endl;
    }else{
        gzmsg << "GazeboThruster: " + parameter_name + ": using default: "
                << default_value << " " + dimension << endl;
    }
    return var;
}


std::vector<gazebo_thruster::GazeboThruster::Thruster> GazeboThruster::loadThrusters()
{
    // Import all thrusters from a model file (sdf)
    std::vector<Thruster> thrusters;
    sdf::ElementPtr modelSDF = model->GetSDF();

    sdf::ElementPtr pluginElement = modelSDF->GetElement("plugin");
    while (pluginElement)
    {
        if(pluginElement->Get<string>("filename") == "libgazebo_thruster.so")
           break;
        else
            pluginElement = pluginElement->GetNextElement("plugin");
    }

    if (!pluginElement)
    {
        string msg = "GazeboThruster: sdf model loaded the thruster plugin, but it cannot be found in the SDF object.\n";
        msg += "GazeboThruster: expected the thruster plugin filename to be libgazebo_thruster.so\n";
        gzthrow(msg);
    }


    gzmsg << "GazeboThruster: found plugin (filename): " << pluginElement->Get<string>("filename") << endl;
    gzmsg << "GazeboThruster: found plugin (name): " << pluginElement->Get<string>("name") << endl;

    sdf::ElementPtr thrusterElement = pluginElement->GetElement("thruster");
    while(thrusterElement)
    {
        // Load thrusters attributes
        Thruster thruster;
        thruster.name = thrusterElement->Get<string>("name");
        gzmsg << "GazeboThruster: thruster name: " << thruster.name << endl;
        thruster.minThrust = getParameter<double>(thrusterElement,"min_thrust","N",-200);
        thruster.maxThrust = getParameter<double>(thrusterElement,"max_thrust","N",200);
        thruster.effort = 0.0;
        thrusters.push_back(thruster);
        thrusterElement = thrusterElement->GetNextElement("thruster");
    }

    if (thrusters.empty())
    {
        string msg = "GazeboThruster: sdf model loads thruster plugin but has no thruster defined.\n";
        msg += "GazeboThruster: please name the links you want to export as thrusters inside the <plugin> tag: \n";
        msg += "GazeboThruster: <thruster name='thruster::right'> ";
        gzthrow(msg);
    }
    return thrusters;
}


void GazeboThruster::checkThrusters(std::vector<Thruster> thrusters)
{
    // Look for link names that match thrusters names
    for(vector<Thruster>::iterator thruster = thrusters.begin(); thruster != thrusters.end(); ++thruster)
    {
        if( !model->GetLink(thruster->name) )
        {
            string msg = "GazeboThruster: no link name match the thruster name: " + thruster->name +
                    " in gazebo model: " + model->GetName();
            gzthrow(msg);
        }
    }
}


void GazeboThruster::initComNode()
{
    // Initialize communication node and subscribe to gazebo topic
    node = transport::NodePtr(new transport::Node());
    node->Init();
    string topicName = model->GetName() + "/thrusters";
    thrusterSubscriber = node->Subscribe("~/" + topicName,&GazeboThruster::readInput,this);
    gzmsg <<"GazeboThruster: create gazebo topic /gazebo/"+ GzGet((*model->GetWorld()), Name, ())
            + "/" + topicName << endl;

    topicName = model->GetName() + "/compensated_mass";
    compensatedMassSubscriber = node->Subscribe("~/" + topicName,&GazeboThruster::readCompensatedMass,this,true);
    gzmsg <<"GazeboThruster: create gazebo topic /gazebo/"+ GzGet((*model->GetWorld()), Name, ())
            + "/" + topicName << endl;
}


void GazeboThruster::readInput(ThrustersMSG const& thrustersMSG)
{
    // Read buffer and update the thruster effort
    for(int i = 0; i < thrustersMSG->thrusters_size(); ++i)
    {
        bool thrusterFound = false;
        const gazebo_thruster::msgs::Thruster& thrusterCMD = thrustersMSG->thrusters(i);
        for(vector<Thruster>::iterator thruster = thrusters.begin();
                thruster != thrusters.end(); ++thruster)
        {
            if(thrusterCMD.name() == thruster->name)
            {
                thrusterFound = true;
                thruster->effort = updateEffort(thrusterCMD);
                checkThrustLimits(thruster);
            }
        }
        if(!thrusterFound)
            gzmsg << "GazeboThruster: incoming thruster name: "<< thrusterCMD.name() << ", not found." << endl;
    }
}

void GazeboThruster::readCompensatedMass(CompMassMSG const& compMassMSG)
{
    gazebo_underwater::Matrix6 matrix(compMassMSG->matrix() - gazebo_underwater::Matrix6::Identity());
    Vector3d cog(gazebo::msgs::ConvertIgn(compMassMSG->cog()));
    if(mass_matrix != matrix)
    {
        updateCompensatedEffort(matrix, cog);
        mass_matrix = matrix;
    }
}

double GazeboThruster::updateEffort(gazebo_thruster::msgs::Thruster thrusterCMD)
{
    if( thrusterCMD.has_effort() ){
        return thrusterCMD.effort();
    }else{
        return 0;
    }
}

void GazeboThruster::checkThrustLimits(vector<Thruster>::iterator thruster)
{
    if(thruster->effort < thruster->minThrust)
    {
        gzmsg << "GazeboThruster: thruster effort below the minimum: " << thruster->minThrust << endl;
        gzmsg << "GazeboThruster: using minThrust: " << thruster->minThrust << ", instead. " << endl;
        thruster->effort = thruster->minThrust;
    }

    if(thruster->effort > thruster->maxThrust)
    {
        gzmsg << "GazeboThruster: incoming thruster effort above the maximum: " << thruster->maxThrust << endl;
        gzmsg << "GazeboThruster: using maxThrust: " << thruster->maxThrust << ", instead. " << endl;
        thruster->effort = thruster->maxThrust;
    }
}


void GazeboThruster::updateBegin(common::UpdateInfo const& info)
{
    for(vector<Thruster>::iterator thruster = thrusters.begin();
            thruster != thrusters.end(); ++thruster)
    {
        physics::LinkPtr link = model->GetLink( thruster->name );
        link->AddLinkForce( thruster->effort*Vector3d::UnitX);
        link->AddLinkForce( thruster->effort*thruster->added_mass_compensated_direction,
                thruster->added_mass_compensated_position );
    }
}

void GazeboThruster::updateCompensatedEffort(gazebo_underwater::Matrix6 const& matrix, Vector3d const& cog)
{
    for(vector<Thruster>::iterator thruster = thrusters.begin();
            thruster != thrusters.end(); ++thruster)
    {
        physics::LinkPtr link = model->GetLink( thruster->name );
        Vector3d force(GzGetIgn((*link), RelativePose, ()).Rot().RotateVector(Vector3d::UnitX));
        gazebo_underwater::Vector6 effort( force, (GzGetIgn((*link), RelativePose, ()).Pos()-cog).Cross(force));
        effort = matrix * effort;
        thruster->added_mass_compensated_direction = GzGetIgn((*link), RelativePose, ()).Rot().RotateVectorReverse(effort.top);
        thruster->added_mass_compensated_position = effort.top.Cross(effort.bottom)/(thruster->added_mass_compensated_direction.SquaredLength()) + cog - GzGetIgn((*link), RelativePose, ()).Pos();
        thruster->added_mass_compensated_position = GzGetIgn((*link), RelativePose, ()).Rot().RotateVectorReverse(thruster->added_mass_compensated_position);
    }
}
