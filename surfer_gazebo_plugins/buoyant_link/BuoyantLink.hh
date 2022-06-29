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

#ifndef SYSTEM_PLUGIN_BUOYANTLINK_HH_
#define SYSTEM_PLUGIN_BUOYANTLINK_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <ignition/gazebo/System.hh>
// It's good practice to use a custom namespace for your project.
namespace ignition
{
    
namespace gazebo {

//inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

namespace systems {

  class BuoyantLinkPrivateData;
  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
  // plugins that want to send commands.
  class BuoyantLink:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
   
  public: BuoyantLink();
  public: ~BuoyantLink() override;
  
    public: void Configure(const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &_eventMgr) override;      
    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
    
      // ID of link entity

    private: std::unique_ptr<BuoyantLinkPrivateData> dataPtr;
  };

}                                                           // close systems
//}                                                          // close gazebo version
}                                                           // close gazebo
}                                                           // close ignition
#endif
