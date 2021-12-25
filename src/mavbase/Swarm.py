#!/usr/bin/env python3

import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Bool
import math
import time
import numpy as np

TOL = 0.5
TOL_GLOBAL = 0.000005
MAX_TIME_DISARM = 15
ALT_TOL = 0.1
DEBUG = False

### mavros_params.yaml ###
mavros_local_position_pub = rospy.get_param("/mavros_local_position_pub")
mavros_velocity_pub= rospy.get_param("/mavros_velocity_pub")
mavros_local_atual = rospy.get_param("/mavros_local_atual")
mavros_state_sub = rospy.get_param("/mavros_state_sub")
mavros_arm = rospy.get_param("/mavros_arm")
mavros_set_mode = rospy.get_param("/mavros_set_mode")
mavros_battery_sub = rospy.get_param("/mavros_battery_sub")
extended_state_sub = rospy.get_param("/extended_state_sub")
mavros_pose_target_sub = rospy.get_param("/mavros_pose_target_sub")
mavros_global_position_sub = rospy.get_param("/mavros_global_position_sub")
mavros_set_global_pub = rospy.get_param("/mavros_set_global_pub")


class Bee:
    def __init__(self, index):
        self.desired_state = ""
        self.drone_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.goal_vel = TwistStamped()
        self.drone_state = State()
        self.battery = BatteryState()
        self.global_pose = NavSatFix()
        self.gps_target = GeoPoseStamped()
 
        '''
        Publishers
        '''
        self.local_position_pub     = rospy.Publisher("/uav{}{}".format(index, mavros_local_position_pub), PoseStamped, queue_size = 20)
        self.velocity_pub           = rospy.Publisher("/uav{}{}".format(index, mavros_velocity_pub),  TwistStamped, queue_size=5)
        self.target_pub             = rospy.Publisher("/uav{}{}".format(index, mavros_pose_target_sub), PositionTarget, queue_size=5)
        self.global_position_pub    = rospy.Publisher("/uav{}{}".format(index, mavros_set_global_pub), GeoPoseStamped, queue_size= 20)

        '''
        Subscribers
        '''

        self.local_atual            = rospy.Subscriber("/uav{}{}".format(index, mavros_local_atual), PoseStamped, self.local_callback)
        self.state_sub              = rospy.Subscriber("/uav{}{}".format(index, mavros_state_sub), State, self.state_callback, queue_size=10) 
        self.battery_sub            = rospy.Subscriber("/uav{}{}".format(index, mavros_battery_sub), BatteryState, self.battery_callback)
        self.global_position_sub    = rospy.Subscriber("/uav{}{}".format(index, mavros_global_position_sub), NavSatFix, self.global_callback)
        self.extended_state_sub     = rospy.Subscriber("/uav{}{}".format(index, extended_state_sub), ExtendedState, self.extended_state_callback, queue_size=2)        
        
        '''
        LANDED_STATE_UNDEFINED = 0
        LANDED_STATE_ON_GROUND = 1
        LANDED_STATE_IN_AIR = 2
        LANDED_STATE_TAKEOFF = 3
        LANDED_STATE_LANDING = 4
        '''

        '''
        Services
        '''
        self.arm                = rospy.ServiceProxy("/uav{}/{}".format(index, mavros_arm), CommandBool)
        self.set_mode_srv       = rospy.ServiceProxy("/uav{}/{}".format(index, mavros_set_mode), SetMode)
        self.LAND_STATE         = ExtendedState.LANDED_STATE_UNDEFINED # landing state

        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.logerr("failed to connect to services")

    """ CALLBACK FUNCTIONS """
    def state_callback(self, state_data):
        self.drone_state = state_data
        if self.drone_state.mode != self.desired_state:
            for i in range (50):
                self.set_mode_srv(0, self.desired_state)

    def battery_callback(self, bat_data):
        self.battery = bat_data

    def local_callback(self, local):
        self.drone_pose.pose.position.x = local.pose.position.x
        self.drone_pose.pose.position.y = local.pose.position.y
        self.drone_pose.pose.position.z = local.pose.position.z

    def extended_state_callback(self, es_data):
        self.LAND_STATE = es_data.landed_state

    def global_callback(self, global_data):
        self.global_pose = global_data

class SWARM:
    def __init__(self, num_drones):
        self.rate = rospy.Rate(60)
        self.mavs = []
        for i in range(num_drones):
            self.mavs.append(Bee(i))

        self.pack_pose = PoseStamped()

    ####### Set Position and Velocity ################
    def set_position(self, x, y, z):
        for mav in self.mavs:
            mav.goal_pose.pose.position.x = x
            mav.goal_pose.pose.position.y = y
            mav.goal_pose.pose.position.z = z
            mav.local_position_pub.publish(mav.goal_pose)
            self.rate.sleep()

    def set_vel(self, x, y, z, roll=0, pitch=0, yaw=0):
        for mav in self.mavs:
            mav.goal_vel.twist.linear.x = x
            mav.goal_vel.twist.linear.y = y
            mav.goal_vel.twist.linear.z = z

            mav.goal_vel.twist.angular.x = roll
            mav.goal_vel.twist.angular.y = pitch
            mav.goal_vel.twist.angular.z = yaw
            mav.velocity_pub.publish(mav.goal_vel)
    
    def set_global_pose(self, mav, lat, lon, altitude, yaw=0):
        mav.gps_target.pose.position.latitude = lat
        mav.gps_target.pose.position.longitude = lon
        mav.gps_target.pose.position.altitude = altitude

        mav.gps_target.pose.orientation.x = mav.gps_target.pose.orientation.y = 0
        mav.gps_target.pose.orientation.z = 1
        mav.gps_target.pose.orientation.w = yaw
        mav.global_position_pub.publish(mav.gps_target)
        self.rate.sleep()
    
    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        for mav in self.mavs: 
            mav.desired_state = mode
            old_mode = mav.drone_state.mode
            loop_freq = 1  # Hz
            loop_rate = rospy.Rate(loop_freq)
            mode_set = False
            for i in range(timeout * loop_freq):
                if mav.drone_state.mode == mode:
                    mode_set = True
                    break 
                else:
                    try:
                        result = mav.set_mode_srv(0, mode)  # 0 is custom mode
                        if not result.mode_sent:
                            rospy.logerr("failed to send mode command")
                    except rospy.ServiceException as e:
                        rospy.logerr(e)

                try:
                    loop_rate.sleep()
                except rospy.ROSException as e:
                    rospy.logerr(e) 

    def takeoff(self, height):
        for mav in self.mavs:
            for i in range(100):
                self.set_position(mav.drone_pose.pose.position.x, mav.drone_pose.pose.position.y, mav.drone_pose.pose.position.z)
                self.rate.sleep()

            self.set_mode("OFFBOARD", 2)
            if not mav.drone_state.armed:
                rospy.logwarn("ARMING DRONES")
                fb = mav.arm(True)
                while not fb.success:
                    rospy.logwarn("ARMING DRONE {}".format(fb))
                    fb = mav.arm(True)
                    self.rate.sleep()
                    rospy.loginfo("DRONES ARMED")
            else:
                rospy.loginfo("DRONE ALREADY ARMED")

            self.rate.sleep()
        rospy.loginfo("STARTING TAKEOFF")
        while abs(self.mavs[0].drone_pose.pose.position.z - height) >= ALT_TOL:
            if DEBUG:
                for mav in self.mavs:
                    rospy.loginfo("TAKEOFF: SETTING ALTITUDE")
                    rospy.logwarn("ALTITUDE: " + str(mav.drone_pose.pose.position.z))
            self.set_position(mav.drone_pose.pose.position.x, mav.drone_pose.pose.position.y, height)
            self.rate.sleep()
        rospy.loginfo("TAKEOFF COMPLETE")
        return "done"

    def RTL(self):
        for mav in self.mavs:
            self.set_mode("AUTO.RTL", 2)

    def hold(self, time):
        now = rospy.Time.now()
        rospy.logwarn("On hold")
        while not rospy.Time.now() - now > rospy.Duration(secs=time):
            for mav in self.mavs:
                mav.local_position_pub.publish(mav.drone_pose)
                self.rate.sleep()

    def land(self):
        velocity = 1
        for mav in self.mavs:
            init_time = rospy.get_rostime().secs
            height = mav.drone_pose.pose.position.z
            self.set_position(mav.drone_pose.pose.position.x, mav.drone_pose.pose.position.y,0)
            self.rate.sleep()
            rospy.logwarn('LANDING')
            while not mav.LAND_STATE == ExtendedState.LANDED_STATE_ON_GROUND or rospy.get_rostime().secs - init_time < (height/velocity)*1.3:
                #if DEBUG:
                #    rospy.logwarn('Landing')
                #    rospy.loginfo('Height: ' + str(abs(mav.drone_pose.pose.position.z)))
                ################## Velocity Control #################
                self.set_vel(0, 0, -velocity, 0, 0, 0)
                self.rate.sleep()
            rospy.logwarn("LANDED_STATE: ON GROUND\nDISARMING")
            mav.arm(False)
        return "succeeded"

    def _disarm(self):
        rospy.logwarn("DISARM MAVs")
        for mav in self.mavs:
            if mav.drone_pose.pose.position.z < TOL:
                for i in range(3):
                    rospy.loginfo('Drone height' + str(self.drone_pose.pose.position.z))
                    mav.arm(False)

    def go_gps_target(self, lat, lon):
        mav0 = self.mavs[0]
        while abs(mav0.global_pose.latitude - lat[0]) >= TOL_GLOBAL or abs(mav0.global_pose.longitude - lon[0]) >= TOL_GLOBAL:
            i = 0
            for mav in self.mavs:
                self.set_global_pose(mav, lat[i], lon[i], self.mavs[0].drone_pose.pose.position.z)
                self.rate.sleep()
                i += 1

    def set_altitude(self, altitude):
        rospy.logwarn("SETTING ALTITUDE TO " + str(altitude) + " m")
        for mav in self.mavs:
            for i in range(100):
                self.set_position(mav.drone_pose.pose.position.x, mav.drone_pose.pose.position.y, altitude)

    def get_cartesian(self, lat,lon):  # conversion from a global position to local coordinates
        lat, lon = np.deg2rad(lat), np.deg2rad(lon)
        R = 6371 # radius of the earth
        x = R * np.cos(lat) * np.cos(lon)
        y = R * np.cos(lat) * np.sin(lon)
        #z = R *np.sin(lat)

        return x,y

    def run_delivery(self, lat, lon):  # function for swarm package delivery

        init_lat = self.mavs[0].global_pose.latitude  # inicial latitude of the fisrt drone in the vector
        init_lon = self.mavs[0].global_pose.longitude  # inicial longitude of the fisrt drone in the vector

        ### CONVERSAO GLOBAL - CARTESIANO ###

        init_x, init_y = self.get_cartesian(init_lat, init_lon)
        final_x, final_y = self.get_cartesian(lat, lon)

        # taking into consideration where the drone inicially is 
        dist_x = final_x - init_x
        sinal_x = 1
        if dist_x < 0:
            sinal_x = -1

        dist_y = final_y - init_y
        sinal_y = -1
        if dist_y < 0:
            sinal_y = 1

        dist_tot = (dist_x**2 + dist_y**2)**(1/2)

        ### PARAMETRIZACAO ###

        velocity = 1
        init_time = rospy.get_rostime().secs

        for i in range(len(self.mavs)):
            rospy.loginfo("GLOBAL POSE OF MAV" + str(i) + ": latitude: " + str(self.mavs[i].global_pose.latitude) + "  longitude: " + str(self.mavs[i].global_pose.longitude))
            rospy.loginfo("LOCAL POSE OF MAV" + str(i) + ": x:" + str(self.mavs[i].drone_pose.pose.position.x) + " y:" + str(self.mavs[i].drone_pose.pose.position.y))
        rospy.loginfo("GOAL POSE: x: " + str(dist_x) + " y:" + str(dist_y))

        actual_x = init_x
        actual_y = init_y
        actual_dist = dist_tot
        rospy.logwarn("DELIVERY TRAJECTORY INITIALIZED")
        while actual_dist >= TOL and not rospy.is_shutdown():
            sec = rospy.get_rostime().secs 
            time = sec - init_time 

            # calculando o polinomio     
            p_x = ((-2 * (velocity**3) * (time**3)) / abs(dist_x)**2) + ((3*(time**2) * (velocity**2))/abs(dist_x))
            p_y = (p_x / abs(dist_x)) * (dist_y)
            self.set_position(sinal_x * p_x, sinal_y * p_y, self.mavs[0].drone_pose.pose.position.z)
            self.rate.sleep()

            actual_x = self.mavs[0].drone_pose.pose.position.x
            actual_y = self.mavs[0].drone_pose.pose.position.y
            calc_x = (dist_x-actual_x)**2
            calc_y = (dist_y - actual_y)**2
            actual_dist = np.sqrt(calc_x + calc_y)
            if DEBUG:
                rospy.loginfo("Pose in x: " + str(actual_x))
                rospy.loginfo("Pose in y: " + str(actual_y))
                rospy.logwarn("actual_dist = " + str(actual_dist))
                rospy.loginfo("dist in x: " + str(abs(dist_x - actual_x)))
                rospy.loginfo("dist in y: " + str(abs(dist_y - actual_y)))
        if DEBUG:
            for mav in self.mavs:
                rospy.loginfo("GLOBAL POSE OF MAV" + str(i) + ": latitude: " + str(self.mavs[i].global_pose.latitude) + "  longitude: " + str(self.mavs[i].global_pose.longitude))
                rospy.loginfo("LOCAL POSE: x:" + str(mav.drone_pose.pose.position.x) + " y:" + str(mav.drone_pose.pose.position.y))
        rospy.logwarn("DELIVERY TRAJECTORY COMPLETE")

        self.rate.sleep()
        self.land()

if __name__ == '__main__':
    mav1 = "mav1"
    mav2 = "mav2"
    swarm = SWARM(mav1, mav2)
    swarm.takeoff(3)
    swarm.land()
