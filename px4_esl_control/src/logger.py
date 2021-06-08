#!/usr/bin/env python2

## logger.py
# Run this as node to set offboard mode, start logging data of uav and save as .csv

# ROS python API
import rospy

# import needed messages
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import * # services
from sensor_msgs.msg import TimeReference

# import quat and eul transformation
from tf.transformations import euler_from_quaternion

# import other system/utils
import time, sys, math

# parameters
activate_offboard = 0 # Set to 1 to automatically activate offboard mode
log_time = 10 # Time to log for [seconds]

# Flight modes class
# Flight modes are activated using ROS services
class FlightModes:
    def __init__(self):
        pass

    # Switch to OFFBOARD mode
    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set."%e)

# Offboard controller for sending setpoints
class Controller:
    def __init__(self):
        # Drone state
        self.state = State()

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)
        self.local_yaw = 0

        # A Message for the current linear velocity of the drone
        self.local_vel = Vector3(0.0, 0.0, 0.0)

    # Callbacks.

    ## Time callback
    def timeCb(self, msg):
        time_ref = msg.time_ref

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Drone local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

        # Yaw: Additional 90 because NED and XYZ is rotated 90 degrees in the horizontal plane (align N and X)
        self.local_yaw = -90 + math.degrees(euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2])

    ## Drone linear velocity callback
    def velCb(self, msg):
        self.local_vel.x = msg.twist.linear.x
        self.local_vel.y = msg.twist.linear.y
        self.local_vel.z = msg.twist.linear.z

def run(argv):
    # initiate node
    rospy.init_node('waypoint_scheduler_node')

    # flight mode object
    modes = FlightModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/time_reference', TimeReference, cnt.timeCb)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Subscribe to drone's linear velocity
    rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, cnt.velCb)

    # activate OFFBOARD mode
    if activate_offboard:
        print("Activate OFFBOARD mode")
        while not (cnt.state.mode == "OFFBOARD" or rospy.is_shutdown()):
            modes.setOffboardMode()
            rate.sleep()
        print("OFFBOARD mode activated\n")

    # Save initial position
    init_pos = Point(cnt.local_pos.x, cnt.local_pos.y, cnt.local_pos.z)

    # ROS main loop
    start_time = rospy.Time.now().to_sec()
    print("Start logging...\n")
    while (not rospy.is_shutdown()): # Run for certain duration and while node is active
        # print("%f, %f, %f, %f, %f, %f, %f, %f" % \
        # (start_time - time.time(),
        # cnt.local_pos.x,
        # cnt.local_pos.y,
        # cnt.local_pos.z,
        # cnt.local_vel.x,
        # cnt.local_vel.y,
        # cnt.local_vel.z,
        # cnt.local_yaw) )  
        d_time = rospy.Time.now().to_sec() - start_time
        print(d_time)     

        
        rate.sleep()

    print("Stop logging\n")

    print("Exit\n")

def main(argv):
    try:
        run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
