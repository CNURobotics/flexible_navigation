#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2016
#  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
#  Christopher Newport University
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#       POSSIBILITY OF SUCH DAMAGE.
###############################################################################

from flexbe_core import EventState

class SetIndiceState(EventState):
    """
    Sets the initial index into a planned path for the Pure Pursuit state

    ># plan         Path       The path.

    #> plan         Path       The path.
    #> indice       int        The index of the current waypoint in the path

    <= done         Successfully did things.

    """

    def __init__(self):
        """
        Constructor
        """
        super(SetIndiceState, self).__init__(outcomes=['done'], input_keys=['plan'], output_keys=['plan', 'indice'])

    def execute(self, userdata):
        """
        Execute this state
        """

        userdata.indice = 1
        return 'done'
