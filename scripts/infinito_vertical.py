#!/usr/bin/env python3

import rospy
from mavbase.MAV import MAV
import numpy as np
import math
TOL = 1

def go():

    rospy.init_node("infinity_node")
    mav = MAV("1")

    takeoff_alt = 5
    t = -math.pi/2
    goal_x = 4*math.cos(t)
    goal_z = 5 + 4*math.cos(t) * math.sin(t)
    # altitude = 2
    # goal_lat = mav.gps_target.pose.position.latitude + 0.00005
    # goal_long = mav.gps_target.pose.position.longitude + 0.00005

    mav.takeoff(takeoff_alt)
    mav.rate.sleep()
    actual_x = mav.drone_pose.pose.position.x
    actual_z = mav.drone_pose.pose.position.z
    actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_z - actual_z)**2)
    rospy.loginfo("Setting position to (%s, 0, %s)" %(str(goal_x), str(goal_z)))
    while t < 3*math.pi/2 and not rospy.is_shutdown():
        mav.set_position(goal_x, 0, goal_z)
        if actual_dist <= TOL:
            t += 0.1
            goal_x = 4*math.cos(t)
            goal_z = 5 + 4*math.cos(t) * math.sin(t)
        actual_x = mav.drone_pose.pose.position.x
        actual_z = mav.drone_pose.pose.position.z
        actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_z - actual_z)**2)
        mav.rate.sleep()
    rospy.loginfo("Infinity completed")
    mav.land()
    mav._disarm()

if __name__ == "__main__":
    go()