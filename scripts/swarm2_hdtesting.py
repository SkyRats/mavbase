#!/usr/bin/env python3

import rospy
import math
import numpy as np
from mavbase.Swarm import SWARM, Bee

#This is a script to test Swarm.py functions
def run_tests():
    rospy.init_node("swarm")
    swarm = SWARM(2)

    takeoff_hgt = 5
    goal_x = 2
    goal_y = 2
    goal_z = takeoff_hgt
    alt = 3

    swarm.takeoff(takeoff_hgt)

    x_refpose = swarm.mavs[0].drone_pose.pose.position.x
    y_refpose = swarm.mavs[0].drone_pose.pose.position.y
    dist = np.sqrt((goal_x - x_refpose)**2 + (goal_y - y_refpose)**2)
    rospy.loginfo("Setting position")
    while not rospy.is_shutdown() and dist > 0.2 :
        rospy.loginfo("dist: " +  str(dist))
        swarm.set_position(goal_x, goal_y, swarm.mavs[0].drone_pose.pose.position.z)
        x_refpose = swarm.mavs[0].drone_pose.pose.position.x
        y_refpose = swarm.mavs[0].drone_pose.pose.position.y
        dist = np.sqrt((goal_x - x_refpose)**2 + (goal_y - y_refpose)**2)
    swarm.set_altitude(alt)
    swarm.hold(5)
    swarm.land()


if __name__ == "__main__":
    run_tests()