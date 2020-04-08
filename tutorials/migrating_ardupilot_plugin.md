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

The old code included these Gazebo-related headers:

```cpp
#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
```

In the new code, we still need `<sdf/sdf.hh>`, because the underlying [SDFormat
library](http://sdformat.org/) is used by both Gazebo and Ignition. But in place of the `<gazebo/...>` headers, we'll pull in one from Ignition:

```cpp
#include <ignition/gazebo/System.hh>
#include <sdf/sdf.hh>
```

In the old code, the plugin class `ArduPilotPlugin` is declared in the `gazebo` namespace:
```cpp
namespace gazebo
{
```

In the new code we declare the class in the `ignition::gazebo::systems` namespace:

```cpp
namespace ignition
{
namespace gazebo
{
namespace systems
{
```

In the old code, the plugin class inherits from `ModelPlugin`:

```cpp
class GAZEBO_VISIBLE ArduPilotPlugin : public ModelPlugin
```

In the new code, we use multiple inheritance to declare that our plugin will
act as a *system* (in the entity-component-system, or ECS, pattern used by
Ignition), and further which interfaces of a system it will use (we also update
the symbol visibility macro:

```cpp
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
virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
```

In the new code, we use `Configure()` for the same purpose (if a different signature):

```cpp
void Configure(const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr);
```

Similarly, the old code provides `OnUpdate()`, which is called once per time step while simulation is running:

```cpp
void OnUpdate();
```

In the new code, this method is replaced by two methods, `PreUpdate()` and
`PostUpdate()`:


```cpp
void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm);

public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm);
```

As the names suggest, the former is called before each time step, while the
latter is called after each time step. Note the subtle difference in signature:
`PreUpdate()` takes a non-`const` reference to the `EntityComponentManager`,
while `PostUpdate()` takes a `const` reference to it. We'll make any changes to
the state of simulation (e.g., setting torques on joints) in `PreUpdate()` and
we'll read out results from simluation (e.g., getting the pose of a link) in
`PostUpdate()`.

The remaining changes in the header are just bookkeeping, to allow us to have
access to the right objects with the right types in other class methods. These three helpers:

```cpp
void ApplyMotorForces(const double _dt);
void SendState();
bool InitArduPilotSockets(sdf::ElementPtr _sdf);
```

become:

```cpp
void ApplyMotorForces(const double _dt,
  ignition::gazebo::EntityComponentManager &_ecm);
void SendState(double _simTime,
  const ignition::gazebo::EntityComponentManager &_ecm);
bool InitArduPilotSockets(const std::shared_ptr<const sdf::Element> &_sdf);
```

