#!/usr/bin/env python

import rospy
import math
from mavbase.Swarm import SWARM, Bee

def run_delivery_test():
    rospy.init_node("swarm")
    swarm = SWARM(2)
    actual_lat = swarm.mavs[0].global_pose.latitude
    actual_lon = swarm.mavs[0].global_pose.longitude
    lat = actual_lat + 0.0008
    lon = actual_lon + 0.0008
    swarm.takeoff(5)
    swarm.run_delivery(lat, lon)



if __name__ == '__main__':
    #rospy.init_node("swarm")
    #mav1 = "mav1"
    #mav2 = "mav2"
    #swarm = SWARM(2)
    ##run_delivery receives the latitude and longitude of the availiable drop zone
    #swarm.run_delivery(19, -98.9)
    run_delivery_test()
    
