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

Migration of this plugin involves modifications to a few parts of the associated code:

1. The plugin header file, `ArduPilotPlugin.hh`.
1. The plugin source file, `ArduPilotPlugin.cc`.
1. The plugin's CMake build recipe, `CMakeLists.txt`.
1. The model...

