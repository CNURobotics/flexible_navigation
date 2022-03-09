Flexible Navigation
===================

NOTE: As of 9-March-2022 development of the ROS 2 version of Flexible Navigation
will be maintained at https://github.com/FlexBE/flexible_navigation.

It is suggested that you migrate to there for new ROS 1 work as well.

This site is being maintained for historical reasons, but future work should use
the https://github.com/FlexBE site, which will be the one-stop shop for all
things FlexBE going forward.

Flexible Navigation is a rework of the ROS Navigation stack into independent modules that
interface with [FlexBE] compatible state implementations.  These allow a user to graphically chain together states in a state machine to essentially create their own navigation stack to suit any need.

This allows for supervisory and sliding autonomy within navigation, and better control over contingencies and recovery behaviors.

About
-----

This system provides specific navigation planning and path following
capabilities based on the ROS Navigation [move_base] packages and is compatible
with any base_global_planner and base_local_planner plugin.

A ROS node wrapper for each plugin type provides several ActionLib
interfaces to [FlexBE] state implementations.

Install
-------

A complete demonstration system is provided as part of the CHRISLab [Turtlebot Flexible Navigation] demonstration. Follow setup and operation directions there for an integrated demonstration.  The complete system is easily setup and built by following the installation directions at [CHRISLab Install].

The [Turtlebot Flexible Navigation] demonstration uses the SBPL lattice planner in a move_base like planning scheme.  An alternative demonstration at [Create Flexible Navigation] uses a three layer planning scheme to demonstrate the flexibility of the decoupled approach.

The Flexible Navigation system has been tested using the latest version of ROS Kinetic. You
should first follow the [ROS Install Guide] and get that set up before proceeding.

## Publications

Please use the following publications for reference when using Flexible Navigation:

- David C. Conner and Justin Willis, ["Flexible Navigation: Finite state machine-based integrated navigation and control for ROS enabled robots,"](http://dx.doi.org/10.1109/SECON.2017.7925266) SoutheastCon 2017.

- Joshua Zutell, David C. Conner and Philipp Schillinger, "ROS 2-Based Flexible Behavior Engine for Flexible Navigation ," to appear, SoutheastCon 2022.


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

[FlexBE]: https://flexbe.github.io
[move_base]: http://wiki.ros.org/move_base
[ROS Install Guide]: http://wiki.ros.org/kinetic/Installation
[Turtlebot Flexible Navigation]: https://github.com/CNURobotics/chris_turtlebot_flexible_navigation
[Create Flexible Navigation]: https://github.com/CNURobotics/chris_create_flexible_navigation
[CHRISLab Install]: https://github.com/CNURobotics/chris_install
