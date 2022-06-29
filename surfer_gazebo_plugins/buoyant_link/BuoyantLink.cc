/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// We'll use a string and the ignmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <Eigen/Eigen>
#include <memory>

#include <ignition/plugin/Register.hh>

#include <boost/make_unique.hpp>

#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Sphere.hh>
#include <ignition/math/Box.hh>
#include <ignition/math/Cylinder.hh>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
//#include <ignition/gazebo/components/WorldLinearVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/CenterOfVolume.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Volume.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Model.hh"

#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/Util.hh"

#include "ignition/transport/Node.hh"
#include <sdf/Box.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.

// Don't forget to include the plugin's header.
#include "BuoyantLink.hh"



using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::BuoyantLinkPrivateData
{

    /// \brief Water density [kg/m^3].
    public: double _waterDensity;

    /// \brief Link entity
    public: Entity _linkEntity;
    public: Entity _surfaceEntity;
    public: double _volume;
    public: double _length;
    public: double _width;
    public: double _height;

};


void AddComponents(
  const Entity &_entity,
  EntityComponentManager &_ecm)
{

  if (!_ecm.Component<ignition::gazebo::components::Pose>(_entity))
  {
    _ecm.CreateComponent(_entity, ignition::gazebo::components::Pose());
    ignmsg << "Component Created" << std::endl;
  }
  
}

BuoyantLink::BuoyantLink(){

    this->dataPtr = boost::make_unique<BuoyantLinkPrivateData>();
    
}
BuoyantLink::~BuoyantLink(){
}

void BuoyantLink::Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/
)
{
    
  auto model = ignition::gazebo::Model(_entity);
    
    if (_sdf->HasElement("water_density"))
    {
        this->dataPtr->_waterDensity = _sdf->Get<double>("water_density");
    }else{
    
        this->dataPtr->_waterDensity = 1000; // default to 1000 kg/m^3
    }
    if (!_sdf->HasElement("surface_name"))
    {
        ignwarn << "No Surface Model Specified, Assuming a plane at z = 0 with normal vector ( 0 0 1)";
    } else{
        auto surfaceName = _sdf->Get<std::string>("surface_name");
        this->dataPtr->_surfaceEntity = _ecm.EntityByComponents(components::Name(std::string(surfaceName)),components::Model());
        
    }
    

    
    if (!_sdf->HasElement("link_name"))
    {
        ignerr << "You musk specify a <link_name> for the BuoyantLink"
        << " plugin to act upon";
        return;
    }
    auto linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->_linkEntity = model.LinkByName(_ecm,linkName);
    
    if (!_ecm.HasEntity(this->dataPtr->_linkEntity))
    {
        ignerr << "Link name" << linkName << "does not exist";
        return;
    }

    AddComponents(this->dataPtr->_linkEntity,_ecm);
        
 //   Link link(this->dataPtr->_linkEntity);

 //   std::vector<Entity> collisions = _ecm.ChildrenByComponents(
//        this->dataPtr->_linkEntity, components::Collision());
    
    


}

void BuoyantLink::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  // This is a simple example of how to get information from UpdateInfo.
  std::string msg = "Hello, Jeremy! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";
 
    ignition::gazebo::Link baseLink(this->dataPtr->_linkEntity);

    uint64_t cols = baseLink.CollisionCount(_ecm);
    std::vector<Entity> collisions = baseLink.Collisions(_ecm);

   // auto my_data = coll->Data()->Geom();

    math::Pose3d surf_pose = worldPose(this->dataPtr->_surfaceEntity,_ecm);
    
    
    const components::CollisionElement *coll = _ecm.Component<components::CollisionElement>(collisions.front());
    if (!coll)
    {
        ignerr << "Invalid collision pointer. This shouldn't happen\n";
    }

    math::Pose3d link_pose = worldPose(this->dataPtr->_linkEntity,_ecm);

    double volume;
    double mx = 0;
    double my = 0;
    double mz = 0;
    double fx = 0;
    double fy = 0;
    double fz = 0;

    double depth = (surf_pose.Z()-link_pose.Z());
    
  switch (coll->Data().Geom()->Type())
      {
        case sdf::GeometryType::BOX:
        {
          math::Vector3d sz = coll->Data().Geom()->BoxShape()->Size();
          this->dataPtr->_length = sz.X();
          this->dataPtr->_width = sz.Y();
          this->dataPtr->_height = sz.Z();
          this->dataPtr->_volume = sz.X()*sz.Y()*sz.Z();   
          fz = this->dataPtr->_volume*this->dataPtr->_waterDensity*9.81;
          break;
        }
        case sdf::GeometryType::SPHERE:
        {
           double rad = coll->Data().Geom()->SphereShape()->Radius();
          break;
        }
        case sdf::GeometryType::CYLINDER:
        {
            double len = coll->Data().Geom()->CylinderShape()->Length();
          break;
        }
        default:
        {
          ignerr << "Unsupported collision geometry["
            << static_cast<int>(coll->Data().Geom()->Type()) << "]\n";
          break;
        }
      }    
      
        double buoyancy_factor = 1/(1+exp(-(8*depth)));

        mx = -buoyancy_factor*fz*this->dataPtr->_length*sin(link_pose.Roll())/2;
        my = -buoyancy_factor*fz*this->dataPtr->_length*sin(link_pose.Pitch())/2;
   

     //   ignmsg << link_pose << std::endl;
    ignition::math::Vector3d totalForce(0,0,fz*buoyancy_factor);
    ignition::math::Vector3d totalTorque(mx,my,0);

    baseLink.AddWorldWrench(_ecm,totalForce,link_pose.Rot()*(totalTorque));    
  //  baseLink.AddWorldWrench(_ecm,totalForce,link_pose.Rot()*(totalTorque));
    //baseLink.A(_ecm, link_pose.Rot()*(totalForce));
    

  // Messages printed with ignmsg only show when running with verbosity 3 or
  // higher (i.e. ign gazebo -v 3)
    //ignmsg << link_pose.Pos() << " " << Fz << std::endl;
}
// Here we implement the PostUpdate function, which is called at every
// iteration.

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    BuoyantLink,
    System,
    BuoyantLink::ISystemConfigure,
    BuoyantLink::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(BuoyantLink,
                          "ignition::gazebo::systems::BuoyantLink")
