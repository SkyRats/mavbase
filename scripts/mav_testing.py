#!/usr/bin/env python3

import rospy
from mavbase.MAV import MAV
import numpy as np

def go():

    rospy.init_node("mav_test")
    mav = MAV("1")

    takeoff_alt = 10
    goal_x = 5
    goal_y = 5
    altitude = 2

    mav.takeoff(takeoff_alt)
    mav.rate.sleep()
    actual_x = mav.drone_pose.pose.position.x
    actual_y = mav.drone_pose.pose.position.y
    actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
    rospy.loginfo("Setting position to (%s, %s)" %(str(goal_x), str(goal_y)))
    while actual_dist > 0.2 and not rospy.is_shutdown():
            mav.set_position(goal_x, goal_y, mav.drone_pose.pose.position.z)
            actual_x = mav.drone_pose.pose.position.x
            actual_y = mav.drone_pose.pose.position.y
            actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
            mav.rate.sleep()
    rospy.loginfo("On hold")
    mav.hold(5)
    rospy.loginfo("Setting altitude to %s m" %(altitude))
    mav.set_altitude(altitude)
    mav.land()
    mav._disarm()

if __name__ == "__main__":
    go()