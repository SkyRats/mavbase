#!/usr/bin/env python

import rospy
import math
from mavbase.Swarm import SWARM, Bee







if __name__ == '__main__':
    rospy.init_node('swarm')
    mav1 = "mav1"
    mav2 = "mav2"
    swarm = SWARM(2)
    swarm.takeoff(3)
    swarm.land()
