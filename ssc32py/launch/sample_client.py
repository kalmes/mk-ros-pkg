#!/usr/bin/python
#
# Copyright 2011, Mark Kalmes
#
# This is just a very simple sample client that shows how to
# make requests against the ROS SSC32 server

import roslib
roslib.load_manifest('ssc32py')
import rospy

import time, math

from ssc32py.ros_ssc32 import ROS_SSC32_Client

if __name__ == '__main__':
    print "Contacting server.."
    
    base = ROS_SSC32_Client('base')
    shoulder = ROS_SSC32_Client('shoulder')
    elbow = ROS_SSC32_Client('elbow')
    wrist = ROS_SSC32_Client('wrist')
    rotate = ROS_SSC32_Client('wrist_rotate')
    
    print "Setup.."
    base.move_angle(math.radians(0), endgroup=False)
    shoulder.move_angle(math.radians(50), endgroup=False)
    elbow.move_angle(math.radians(50), endgroup=False)
    wrist.move_angle(math.radians(-55), endgroup=False)
    rotate.move_angle(math.radians(0), timesecs=1)
    while base.is_moving(): time.sleep(0.1)
    print "Complete!"
    
    print "Moving to first position.."
    base.move_angle(math.radians(-55), endgroup=False)
    shoulder.move_angle(math.radians(50), endgroup=False)
    elbow.move_angle(math.radians(50), endgroup=False)
    wrist.move_angle(math.radians(-80), endgroup=False)
    rotate.move_angle(math.radians(-20), timesecs=5)
    
    print "Waiting for move to complete.."
    while base.is_moving(): time.sleep(0.1)
    print "Complete!"
    
    time.sleep(2)

    print "Moving to second position.."
    base.move_angle(math.radians(0), endgroup=False)
    shoulder.move_angle(math.radians(50), endgroup=False)
    elbow.move_angle(math.radians(70), endgroup=False)
    wrist.move_angle(math.radians(-55), endgroup=False)
    rotate.move_angle(math.radians(0), timesecs=5)
   
    print "Waiting for move to complete.."
    while base.is_moving(): time.sleep(0.1)
    print "Complete!"
    
    time.sleep(2)
    
    print "Moving to third position.."
    base.move_angle(math.radians(55), endgroup=False)
    shoulder.move_angle(math.radians(50), endgroup=False)
    elbow.move_angle(math.radians(50), endgroup=False)
    wrist.move_angle(math.radians(-80), endgroup=False)
    rotate.move_angle(math.radians(20), timesecs=5)
    
    print "Waiting for move to complete.."
    while base.is_moving(): time.sleep(0.1)
    print "Complete!"
    
