---
tags:
    - gazebo classic
    - plugin
    - joint
    - velocity
---

# SetVelocity

```cpp
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class JointControllerPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&JointControllerPlugin::OnUpdate, this));
    }

    // Called by the world update start event
  public:
    void OnUpdate()
    {
      this->model->GetJoint("joint_1")->SetVelocity(0, 2.0);
    }

    // Pointer to the model
  private:
    physics::ModelPtr model;

    // Pointer to the update event connection
  private:
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(JointControllerPlugin)
}
```