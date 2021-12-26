#!/usr/bin/env python3

from numpy.core.fromnumeric import take
import rospy
from mavbase.MAV import MAV
import numpy as np
import math
TOL = 2

def go():

    rospy.init_node("S_node")
    mav = MAV("1")

    # parameterizes below
    takeoff_alt = 16
    t = math.pi/1.85
    goal_x = -4*math.sinh(2*t)*math.cos(t)
    goal_z = 10 + 0.5*math.cosh(2*t)*math.sin(t)
    ##################

    # sets variables and MAV up
    mav.takeoff(takeoff_alt)
    rospy.loginfo("Setting MAV's height to %s" %(str(takeoff_alt)))
    mav.rate.sleep()
    actual_x = mav.drone_pose.pose.position.x
    actual_z = mav.drone_pose.pose.position.z
    actual_dist = math.sqrt((goal_x - actual_x)**2 + (goal_z - actual_z)**2)
    ###########################

    #initiates S drawing
    rospy.loginfo("Setting MAV's position to (%s, 0, %s)" %(str(goal_x), str(goal_x)))

    while t > -math.pi/1.85 and not rospy.is_shutdown():
        
        mav.set_position(goal_x, 0, goal_z)
        
        if actual_dist <= TOL:

            if t >= 1.0 or t <= -1.0:
                t -= 0.01
            else:
                t -= 0.035

            goal_x = -4*math.sinh(2*t)*math.cos(t)
            goal_z = 10 + 0.5*math.cosh(2*t)*math.sin(t)
            rospy.loginfo("Setting MAV's position to (%s, 0, %s)" %(str(goal_x), str(goal_x)))

        actual_x = mav.drone_pose.pose.position.x
        actual_z = mav.drone_pose.pose.position.z
        actual_dist = math.sqrt((goal_x - actual_x)**2 + (goal_z - actual_z)**2)

    
    mav.hold(2)
    mav.land()
    mav._disarm()
    rospy.loginfo("Mission terminated successfully")

if __name__ == "__main__":
    go()
