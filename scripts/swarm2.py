#!/usr/bin/env python

import rospy
import math
from mavbase.Swarm import SWARM, Bee







if __name__ == '__main__':
    rospy.init_node("swarm")
    mav1 = "mav1"
    mav2 = "mav2"
    swarm = SWARM(2)
    #run_delivery receives the latitude and longitude of the availiable drop zone
    swarm.run_delivery(19, -98.9)
    
