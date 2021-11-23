#!/usr/bin/env python3

from numpy.core.fromnumeric import take
import rospy
from mavbase.MAV import MAV
import numpy as np
import math
TOL = 2

def go():

    rospy.init_node("mav_test")
    mav = MAV("1")

    takeoff_alt = 10
    land_altitude = 2

    t = math.pi/1.85
    # goal_x = (-7)*math.sinh(2*t)*math.cos(t)
    # goal_y = math.cosh(2*t) * math.sin(t)
    goal_x = 0
    goal_y = 14.8
    actual_x = mav.drone_pose.pose.position.x
    actual_y = mav.drone_pose.pose.position.y

    mav.takeoff(takeoff_alt)
    mav.rate.sleep()
    rospy.loginfo("Setting position to S(%s, %s)" %(str(goal_x), str(goal_y)))
    while(np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2) >= TOL):
        actual_x = mav.drone_pose.pose.position.x
        actual_y = mav.drone_pose.pose.position.y
        mav.set_position(0, 14.8, takeoff_alt)
        mav.rate.sleep()
    mav.rate.sleep()
    

    # letter S
    inclination = (2*math.sin(t)*math.sinh(2*t) + math.cos(t)*math.cosh(2*t))/(7*math.sin(t)*math.sinh(2*t) - 14*math.cos(t)*math.cosh(2*t))
    
    inclination = math.atan2((2*math.sin(t)*math.sinh(2*t) + math.cos(t)*math.cosh(2*t)), (7*math.sin(t)*math.sinh(2*t) - 14*math.cos(t)*math.cosh(2*t)))
    inclination = math.atan2(goal_y - actual_y, goal_x - actual_x)
    # goal_x_vel = 2*math.cos(inclination)
    # goal_y_vel = 2*math.sin(inclination)
    actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)

    while t > -math.pi/1.85 and not rospy.is_shutdown():
        mav.set_position(goal_x, goal_y, takeoff_alt)
    #     mav.set_vel(goal_x_vel, goal_y_vel, 0)
        
        actual_x = mav.drone_pose.pose.position.x
        actual_y = mav.drone_pose.pose.position.y

        if np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2) <= TOL:
            t -= 0.2
            goal_x = (-7)*math.sinh(2*t)*math.cos(t)
            goal_y = math.cosh(2*t) * math.sin(t)
            inclination = math.atan2((2*math.sin(t)*math.sinh(2*t) + math.cos(t)*math.cosh(2*t)), (7*math.sin(t)*math.sinh(2*t) - 14*math.cos(t)*math.cosh(2*t)))
            goal_x_vel = 2*math.cos(inclination)
            goal_y_vel = 2*math.sin(inclination)
            rospy.loginfo("Setting position to S(%s, %s)" %(str(goal_x), str(goal_y)))

    #     actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
        mav.rate.sleep()

    #end letter S


    # offset = 7.5

    # letter K
    # t = math.pi/1.85
    # goal_x = (-7)*math.sinh(2*t)*math.cos(t) + offset
    # goal_y = math.sinh(2*t) * math.sin(t) + offset
    # actual_dist = np.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
    # rospy.loginfo("Setting position to K(%s, %s)" %(str(goal_x), str(goal_y)))
    # mav.set_position(goal_x, goal_y, takeoff_alt)
    # mav.rate.sleep()
    # mav.set_position(goal_x, goal_y - 29.6, takeoff_alt)
    # mav.rate.sleep()
    # mav.set_position(goal_x, goal_y - 14.8, takeoff_alt)
    # mav.rate.sleep()
    # mav.set_position(goal_x + 7.5, goal_y, takeoff_alt)
    # mav.rate.sleep()
    # mav.set_position(goal_x, goal_y - 14.8, takeoff_alt)
    # mav.rate.sleep()
    # mav.set_position(goal_x + 7.5, goal_y - 29.6, takeoff_alt)
    # mav.rate.sleep()

    #end letter K
    
    rospy.loginfo("On hold")
    mav.hold(5)
    rospy.loginfo("Setting altitude to %s m" %(land_altitude))
    mav.set_altitude(land_altitude)
    mav.land()
    mav._disarm()

if __name__ == "__main__":
    go()
