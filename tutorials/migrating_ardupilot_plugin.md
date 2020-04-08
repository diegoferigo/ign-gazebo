# Case study: migrating the ArduPilot ModelPlugin from Classic Gazebo to Ignition Gazebo

A variety of changes are required when migrating a plugin from Gazebo Classic
("Gazebo") to Ignition Gazebo ("Ignition"). In this tutorial we offer as a case
study the migration of one particular `ModelPlugin`,
[ardupilot_gazebo](https://github.com/khancyr/ardupilot_gazebo). We hope that
this example provides useful tips to others who are migrating their existing
plugins from Gazebo to Ignition.

The migrated version of the plugin that we're presenting here can be found in
[this fork](https://github.com/gerkey/ardupilot_gazebo/tree/ignition).

## Background

The `ardupilot_gazebo` plugin is used with Gazebo to assist with simulating
unmanned aerial vehicles (UAVs, aka drones). The plugin works like this...

**TODO: Create, insert, and describe system diagram showing how the plugin
talks to Gazebo and the external ArduPilot process, and then to QGC or other
MAVlink stuff.**

## Structure of the migration

Migration of this plugin involves modifications to multiple parts of the associated code:

1. The plugin header file, `ArduPilotPlugin.hh`
1. The plugin source file, `ArduPilotPlugin.cc`
1. The plugin's CMake build recipe, `CMakeLists.txt`
1. The custom model in which the plugin is used

We'll take them each in turn in the following sections.

## Plugin header file (ArduPilotPlugin.hh)

### Headers

The old code includes these Gazebo-related headers:

```cpp
// OLD
#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
```

In the new code, we still need `<sdf/sdf.hh>`, because the underlying [SDFormat
library](http://sdformat.org/) is used by both Gazebo and Ignition. But in place of the `<gazebo/...>` headers, we'll pull in one from Ignition:

```cpp
// NEW
#include <ignition/gazebo/System.hh>
#include <sdf/sdf.hh>
```

### Class declaration

In the old code, the plugin class `ArduPilotPlugin` is declared in the `gazebo` namespace:
```cpp
// OLD
namespace gazebo
{
```

In the new code we declare the class in the `ignition::gazebo::systems` namespace:

```cpp
// NEW
namespace ignition
{
namespace gazebo
{
namespace systems
{
```

In the old code, the plugin class inherits from `ModelPlugin`:

```cpp
// OLD
class GAZEBO_VISIBLE ArduPilotPlugin : public ModelPlugin
```

In the new code, we use multiple inheritance to declare that our plugin will
act as a *system* (in the entity-component-system, or ECS, pattern used by
Ignition), and further which interfaces of a system it will use (we also update
the symbol visibility macro:

```cpp
// NEW
class IGNITION_GAZEBO_VISIBLE ArduPilotPlugin:
       public ignition::gazebo::System,
       public ignition::gazebo::ISystemConfigure,
       public ignition::gazebo::ISystemPostUpdate,
       public ignition::gazebo::ISystemPreUpdate
```

With this declaration we're indicating that our plugin will supply implementation of the `Configure()`, `PreUpdate()`, and `PostUpdate()` methods.

In the old code, the `Load()` method is called once for each instance of the
plugin that is loaded, allowing for startup configuration, like pulling
parameters out of the plugin's SDF configuration:

```cpp
// OLD
virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
```

In the new code, we use `Configure()` for the same purpose (if a different signature):

```cpp
// NEW
void Configure(const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr);
```

Similarly, the old code provides `OnUpdate()`, which is called once per time step while simulation is running:

```cpp
// OLD
void OnUpdate();
```

In the new code, this method is replaced by two methods, `PreUpdate()` and
`PostUpdate()`:


```cpp
// NEW
void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm);

void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm);
```

As the names suggest, the former is called before each time step, while the
latter is called after each time step. Note the subtle difference in signature:
`PreUpdate()` takes a non-`const` reference to the `EntityComponentManager`,
while `PostUpdate()` takes a `const` reference to it. We'll make any changes to
the state of simulation (e.g., setting torques on joints) in `PreUpdate()` and
we'll read out results from simulation (e.g., getting the pose of a link) in
`PostUpdate()`.

The remaining changes in the header are just bookkeeping, to allow us to have
access to the right objects with the right types in other class methods. These three helpers:

```cpp
// OLD
void ApplyMotorForces(const double _dt);
void SendState();
bool InitArduPilotSockets(sdf::ElementPtr _sdf);
```

become:

```cpp
// NEW
void ApplyMotorForces(const double _dt,
  ignition::gazebo::EntityComponentManager &_ecm);
void SendState(double _simTime,
  const ignition::gazebo::EntityComponentManager &_ecm);
bool InitArduPilotSockets(const std::shared_ptr<const sdf::Element> &_sdf);
```

## Plugin source file (ArduPilotPlugin.cc)

### Headers

The old code includes these Gazebo-related headers:

```cpp
// OLD
#include <sdf/sdf.hh>
#include <ignition/math/Filter.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
```

Like we did in `ArduPilotPlugin.hh`, we'll keep `<sdf/sdf.hh>`. The others are
replaced with Ignition equivalents, and where possible we narrow the inclusion
to exactly what we need. We start by enumerating those *components* (part of the
ECS pattern used by Ignition) that we're using:

```cpp
// NEW
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
```

Then we include the parts of `ign-gazebo` itself that we're using:

```cpp
// NEW
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
```

We need a few things from `ign-math`:

```cpp
// NEW
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/Vector3.hh>
```
 
To use the `IGNITION_ADD_PLUGIN()` and `IGNITION_ADD_PLUGIN_ALIAS()` macros, we
need a header from `ign-plugin`:

```cpp
// NEW
#include <ignition/plugin/Register.hh>
```

Because we'll be subscribing to data published by a sensor, we need a header from `ign-transport`:

```cpp
// NEW
#include <ignition/transport/Node.hh>
```

And we keep the SDFormat header:

```cpp
// NEW
#include <sdf/sdf.hh>
```

### Class members

Now let's get into the class member declarations. The `PID` class has moved from `common`:

```cpp
// OLD
common::PID pid;
```

to `ignition::math`:

```cpp
// NEW
ignition::math::PID pid;
```

In the old code we store a `physics::JointPtr` for each propeller joint we're controlling:

```cpp
// OLD
physics::JointPtr joint;
```

In the new code we store an `ignition::gazebo::Entity` instead:

```cpp
// NEW
ignition::gazebo::Entity joint;
```

In the old code we store an `event::ConnectionPtr` to manage periodic calls to the `OnUpdate()` method:

```cpp
// OLD
event::ConnectionPtr updateConnection;
```

There's no equivalent class member in the new code.

In the old code we store a `physics::ModelPtr` for the model we're acting on:

```cpp
// OLD
physics::ModelPtr model;
```

In the new code we instead store references to the model, the entity underlying
the model, and the entity underyling one of the links in the model:

```cpp
// NEW
ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};
ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
ignition::gazebo::Entity modelLink{ignition::gazebo::kNullEntity};
```

The old code uses a custom time class:

```cpp
// OLD
gazebo::common::Time lastControllerUpdateTime;
```

while the new code uses `std::chrono`:

```cpp
// NEW
std::chrono::steady_clock::duration lastControllerUpdateTime{0};
```

In this plugin we need to read data from an IMU sensor attached to the UAV. In
the old code we store a pointer to the sensor:

```cpp
// OLD
sensors::ImuSensorPtr imuSensor;
```

In the new code, instead of accessing the sensor object directly we must
subscribe to a topic published by the sensor (you might be tempted to try
retrieving the sensor data via components attached to the IMU entity, but that
won't work because the logic to produce the data lives in the IMU system and
its output can only be consumed via subscription). So we need a few more
variables to track the state of subscription, data receipt via subscription,
and so on:

```cpp
// NEW
std::string imuName;
bool imuInitialized;
ignition::transport::Node node;
ignition::msgs::IMU imuMsg;
bool imuMsgValid;
std::mutex imuMsgMutex;
```

We also need a callback function that will be invoked upon receipt of newly
published data from the IMU sensor. The callback just latches the latest
message in a mutex-controlled fashion:

```cpp
// NEW
void imuCb(const ignition::msgs::IMU &_msg)
{
  std::lock_guard<std::mutex> lock(this->imuMsgMutex);
  imuMsg = _msg;
  imuMsgValid = true;
}
```

### Console logging

Throughout the code, we replace the following output streams from the old code

```cpp
// OLD
gzdbg << ... ;
gzlog << ... ;
gzwarn << ... ;
gzerr << ... ;
```

with their Ignition equivalents (perhaps the old versions could stick around
and be deprecated instead of removed?):

```cpp
// NEW
igndbg << ... ;
ignlog << ... ;
ignwarn << ... ;
ignerr << ... ;
```

### Class methods: Configure()

Recall that `Configure()` replaces `Load()`.

In the old code, we store the model pointer and name:

```cpp
// OLD
this->dataPtr->model = _model;
this->dataPtr->modelName = this->dataPtr->model->GetName();
```

In the new code, we store the entity, model, and name a bit differently:

```cpp
// NEW
this->dataPtr->entity = _entity;
this->dataPtr->model = ignition::gazebo::Model(_entity);
this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);
```

Also in the new code we clone the `const sdf::Element` that we're passed so
that we can call non-`const` methods on it:

```cpp
// NEW
auto sdfClone = _sdf->Clone();
```

In the old code we retrieve a pointer to each joint that we're controlling:

```cpp
// OLD
control.joint = _model->GetJoint(control.jointName);
```

In the new code we retrieve the entity that represents the joint:

```cpp
// NEW
control.joint = this->dataPtr->model.JointByName(_ecm, control.jointName);
```

The accessor methods for members in the `PID` class have changed. The old code uses a `Get` prefix, e.g.:

```cpp
// OLD
param = controlSDF->Get("vel_p_gain", control.pid.GetPGain()).first;
param = controlSDF->Get("vel_i_gain", control.pid.GetIGain()).first;
param = controlSDF->Get("vel_d_gain", control.pid.GetDGain()).first;
```

In the new code, the `Get` prefix is gone (perhaps the old methods could stick
around and be deprecated instead of removed?):

```cpp
// NEW
param = controlSDF->Get("vel_p_gain", control.pid.PGain()).first;
param = controlSDF->Get("vel_i_gain", control.pid.IGain()).first;
param = controlSDF->Get("vel_d_gain", control.pid.DGain()).first;
```

The old code does a bunch of lookups to get a pointer to the IMU sensor. In the
new code, we just store the name of the sensors from the user-supplied SDF
configuration:

```cpp
// NEW
this->dataPtr->imuName = _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
```

and we do the equivalent lookup later, in `PreUpdate()`, which we'll cover next.

### Class methods: PreUpdate()


