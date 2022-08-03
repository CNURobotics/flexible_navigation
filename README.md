Flexible Navigation
===================

Flexible Navigation is a rework of the ROS2 Navigation2 stack into independent modules that
interface with [FlexBE App] and [FlexBE Behavior Engine] compatible state implementations.  
These allow a user to graphically chain together states in a state machine to essentially
create their own navigation stack to suit any need.

This allows for supervisory and sliding autonomy within navigation, and better
control over contingencies and recovery behaviors.

About
-----

This system provides specific navigation planning and path following
capabilities based on the ROS2 Navigation2 [Nav2] packages and is compatible
with any global_planner and local_planner plugin.

A ROS node wrapper for each plugin type provides several Nav2's simple action server
interfaces to [FlexBE App] state implementations.

Install
-------

Clone this repository to your ROS workspace.

<pre>
rosdep update
rosdep install --from-paths src --ignore-src
</pre>

This will install the necessary dependencies, primarily those associated with the ROS [Navigation2] system.
system.

Then, from the ROS workspace folder do the normal
<pre>
colcon build
</pre>

A complete demonstration setup for the system is provided at [Turtlebot Flexible Navigation].
Follow setup and operation directions there for an integrated demonstration.  


The [Turtlebot Flexible Navigation] provides two demonstrations.
The first uses a 2-level planner as a demonstration, where the
global planner plans over the full map, and a local planner plans over smaller window.

An alternative demonstration uses a three layer planning scheme to demonstrate the flexibility of the decoupled approach.

The Flexible Navigation system has been tested using the latest version of ROS2 Foxy on Ubuntu 20.04.
You should first follow the [ROS2 Install Guide] and get that set up before proceeding.

## Publications

Please use the following publications for reference when using Flexible Navigation:

- David C. Conner and Justin Willis, ["Flexible Navigation: Finite state machine-based integrated navigation and control for ROS enabled robots,"](http://dx.doi.org/10.1109/SECON.2017.7925266) IEEE SoutheastCon 2017.

- Joshua Zutell, David C. Conner and Philipp Schillinger, ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation ,"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047), IEEE SouthEastCon 2022.

### Further Publications for FlexBE

- Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

- Stefan Kohlbrecher et al. ["A Comprehensive Software Framework for Complex Locomotion and Manipulation Tasks Applicable to Different Types of Humanoid Robots."](http://dx.doi.org/10.3389/frobt.2016.00031) Frontiers in Robotics and AI 3 (2016): 31.

- Alberto Romay et al., [“Collaborative autonomy between high-level behaviors and human operators for remote manipulation tasks using different humanoid robots,”](http://dx.doi.org/10.1002/rob.21671) Journal of Field Robotics, September 2016.


License
-------

	Copyright (c) 2016-2022
	Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
	Christopher Newport University

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	  1. Redistributions of source code must retain the above copyright notice,
	     this list of conditions and the following disclaimer.

	  2. Redistributions in binary form must reproduce the above copyright
	     notice, this list of conditions and the following disclaimer in the
	     documentation and/or other materials provided with the distribution.

	  3. Neither the name of the copyright holder nor the names of its
	     contributors may be used to endorse or promote products derived from
	     this software without specific prior written permission.

	     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	     COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
	     WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	     POSSIBILITY OF SUCH DAMAGE.

[FlexBE App]: https://github.com/FlexBE/flexbe_app.git
[FlexBE Behavior Engine]: https://github.com/FlexBE/flexbe_behavior_engine.git
[Navigation2]: https://github.com/ros-planning/navigation2
[ROS2 Install Guide]: https://docs.ros.org/en/foxy/Installation.html
[Turtlebot Flexible Navigation]: https://github.com/FlexBE/flex_nav_turtlebot3_demo
