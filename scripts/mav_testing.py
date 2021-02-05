#!/usr/bin/env python

import rospy
from mavbase.MAV import MAV
import numpy as np
TOL = 0.2

def go():

    rospy.init_node("mav_test")
    mav = MAV("1")
    testing = True
    takeoff_alt = float(input("Enter takeoff altitude (in meters): "))
    goal_x = float(input("Enter the position in x: "))
    goal_y = float(input("Enter the position in y: "))
    mav.takeoff(takeoff_alt)
    mav.rate.sleep()
    actual_x = mav.drone_pose.pose.position.x
    actual_y = mav.drone_pose.pose.position.y
    actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
    rospy.loginfo("Setting position to (%s, %s)" %(str(goal_x), str(goal_y)))
    while actual_dist > TOL and not rospy.is_shutdown():
            mav.set_position(goal_x, goal_y, mav.drone_pose.pose.position.z)
            actual_x = mav.drone_pose.pose.position.x
            actual_y = mav.drone_pose.pose.position.y
            actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
    mav.rate.sleep()
    mav.land()

if __name__ == "__main__":
    go()
