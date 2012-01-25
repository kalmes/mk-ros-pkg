#!/usr/bin/python
#
# Copyright 2011, Mark Kalmes
# portions Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Mark Kalmes nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY MARK KALMES ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# ros_ssc32 provides both a sample Server for a SSC-32 controller and 
# a sample Client to interact with the server

# ROS imports
import roslib
roslib.load_manifest('ssc32py')
import rospy

from std_msgs.msg import Float64, Float32
from ssc32py.srv import None_Float
from ssc32py.srv import None_FloatResponse
from ssc32py.srv import MoveAng
from ssc32py.srv import MoveAngResponse
from ssc32py.srv import None_Int32
from ssc32py.srv import None_Int32Response

from ssc32py.lib_ssc32 import SSC32_Controller, SSC32_Servo
import time
import math
from threading import Thread


class ROS_SSC32_Server():
    # This class provides ROS services for a single servo attached to the SSC32
    def __init__(self, servo, name):

        self.servo = servo
        self.name = name
        
        rospy.logout( 'ROS_SSC32_Server: Starting servo/' + self.name )
        self.channel = rospy.Publisher('servo/' + self.name, Float32)

        self.__service_ang = rospy.Service('servo/' + name + '/readangle',
                                           None_Float, self.__cb_readangle)

        self.__service_ismove = rospy.Service('servo/' + name + '/ismoving',
                                              None_Int32, self.__cb_ismoving)

        self.__service_moveang = rospy.Service('servo/' + name + '/moveangle',
                                               MoveAng, self.__cb_moveangle)

    def __cb_readangle( self, request ):
        ang = self.update_server()
        return None_FloatResponse( ang )

    def __cb_ismoving( self, request ):
        '''actually we can't tell if an individual servo is moving - 
            just querying the controller'''
        status = self.servo.ssc32.is_moving()
        return None_Int32Response( int(status) )

    def __cb_moveangle( self, r ):
        self.servo.move_angle(r.angle, r.angvel, r.timesecs, r.endgroup)
        return MoveAngResponse()

    def update_server(self):
        ang = self.servo.read_angle()
        self.channel.publish( Float32(ang) )
        return ang


class ROS_SSC32_Poller( Thread ):
    # A utility class that will set up and poll a number of ROS_SSC32_Servos -
    # expects to find configuration values in ROS parameters
    def __init__(self, servo_config=None):
        '''loads list of servos and configuration from private parameters 
             - or can optionally be passed a dictionary directly in servo_config
             
            config should contain:
                ssc32/dev_name, ssc32/dev_baud, ssc32/poll_delay: 
                        basic setup for the controller,
                        these default to (/dev/ttyUSB0, 115200, 0.01)
                base, shoulder, elbow... : list of servos as remaining keys
                base/id, base/min_ang, base/max_ang... : 
                        config for each servo, details in SSC32_Servo.__init__
        '''
        Thread.__init__(self)

        if servo_config:
            self.config = servo_config
        else:
            self.config = rospy.get_param('~')

        # init basic parameters
        self.should_run = True
        self.dev_name = '/dev/ttyUSB0'
        self.dev_baud = 115200
        self.poll_delay = 0.01
        if 'ssc32' in self.config:
            ssc32 = self.config['ssc32']
            if 'dev_name' in ssc32: self.dev_name = ssc32['dev_name']
            if 'dev_baud' in ssc32: self.dev_baud = ssc32['dev_baud']
            if 'poll_delay' in ssc32: self.poll_delay = ssc32['poll_delay']

        rospy.logwarn('dev_name' + str(self.dev_name))
        self.dyn = SSC32_Controller(self.dev_name, self.dev_baud)

        # set up each servo
        self.names = []
        self.servos = []
        self.ros_servers = []
        for name in self.config:
            if name == 'ssc32': continue
            servo = SSC32_Servo(self.dyn, servo_param=self.config[name])
            self.names.append(name)
            self.servos.append(servo)
            self.ros_servers.append(ROS_SSC32_Server(servo, name))
        
        rospy.logout( 'ROS_SSC32_Poller: Setup Complete on ' + self.dev_name )

        self.start()

    def run( self ):
        while self.should_run and not rospy.is_shutdown():
            [ s.update_server() for s in self.ros_servers ]
            time.sleep(self.poll_delay)

        for n in self.names:
            rospy.logout( 'ROS_SSC32_Poller: Shutting Down servo/' 
                          + n + ' on ' + self.dev_name )

    def stop(self):
        self.should_run = False
        self.join(3)
        if (self.isAlive()):
            raise RuntimeError("ROS_SSC32_Poller: unable to stop thread")



class ROS_SSC32_Client():
    # Provides access to a named servo being served by ROS_SSC32_Server
    def __init__(self, name = '' ):
        self.name = name

        rospy.wait_for_service('servo/' + name + '/readangle')
        rospy.wait_for_service('servo/' + name + '/ismoving')
        rospy.wait_for_service('servo/' + name + '/moveangle')

        self.__service_ang = rospy.ServiceProxy('servo/' + name + '/readangle',
                                                None_Float)

        self.__service_ismoving = rospy.ServiceProxy('servo/' + name + '/ismoving',
                                                   None_Int32)
        
        self.__service_moveang = rospy.ServiceProxy('servo/' + name + '/moveangle',
                                               MoveAng)

    def read_angle( self ):
        resp = self.__service_ang()
        ang = resp.value
        return ang
        
    def is_moving( self ):
        resp = self.__service_ismoving()
        return bool( resp.value )
        
    def move_angle(self, angle, angvel = math.radians(50), 
                   timesecs=0, endgroup=True):
        self.__service_moveang(angle, angvel, timesecs, endgroup)

if __name__ == '__main__':
    
    print 'Starting SSC32 Basic Server'

    # this basic server will assume that servo parameters are set up
    # in the node parameter space, and poll any servos defined there
    rospy.init_node('ssc32_server')
    ROS_SSC32_Poller()


## SAMPLE CLIENTS:
        
#     tilt = ROS_SSC32_Client( 'tilt' )
#     tilt.move_angle( math.radians( 0 ), math.radians(10))
#     while tilt.is_moving():
#         print 'Tilt is moving'

#     pan = ROS_SSC32_Client( 'pan' )
#     pan.move_angle( math.radians( 0 ), math.radians(10))
#     while pan.is_moving():
#         print 'pan is moving'

