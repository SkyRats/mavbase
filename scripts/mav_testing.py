#!/usr/bin/env python3

import rospy
from mavbase.MAV import MAV
import numpy as np
import math
TOL = 0.2

def go():

    rospy.init_node("mav_test")
    mav = MAV("1")

    takeoff_alt = 10
    t = -math.pi/2
    goal_x = 4*math.cos(t)
    goal_y = 4*math.cos(t) * math.sin(t)
    altitude = 2
    # goal_lat = mav.gps_target.pose.position.latitude + 0.00005
    # goal_long = mav.gps_target.pose.position.longitude + 0.00005

    mav.takeoff(takeoff_alt)
    mav.rate.sleep()
    actual_x = mav.drone_pose.pose.position.x
    actual_y = mav.drone_pose.pose.position.y
    actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
    rospy.loginfo("Setting position to (%s, %s)" %(str(goal_x), str(goal_y)))
    while t < 3*math.pi/2 and not rospy.is_shutdown():
        mav.set_position(goal_x, goal_y, takeoff_alt)
        if actual_dist <= TOL:
            t += 0.2
            goal_x = 4*math.cos(t)
            goal_y = 4*math.cos(t) * math.sin(t)
        actual_x = mav.drone_pose.pose.position.x
        actual_y = mav.drone_pose.pose.position.y
        actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
        mav.rate.sleep()
    rospy.loginfo("On hold")
    mav.hold(5)
    rospy.loginfo("Setting altitude to %s m" %(altitude))
    mav.set_altitude(altitude)
    #mav.go_gps_target(goal_lat, goal_long)
    mav.land()
    mav._disarm()

if __name__ == "__main__":
    go()
