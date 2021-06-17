#!/usr/bin/env python2
# ROS python API
import rospy

# import needed geometry messages
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# import quat and eul transformation
from tf.transformations import euler_from_quaternion

# import other system/utils
import time, sys, math

publish_to_mavros = 1

# Flight modes class
# Flight modes are activated using ROS services
class FlightModes:
    def __init__(self):
        pass

    # Arm the vehicle
    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s"%e)

    # Switch to OFFBOARD mode
    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    # Land the vehicle
    def setLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Land Mode could not be set."%e)

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

        # Instantiate the position setpoint message
        self.sp_raw = PositionTarget()

        # set the flag to control N, E, D and yaw
        # type mask: http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html
        # set 0 for each parameter you want to command from your setpoint
        # set 1 for each parameter you want to ignore in your setpoint object
        # [yaw_rate, yaw, acc_sp_is_force, az, ay, ax, vz, vy, vx, z, y, x]
        self.sp_raw.type_mask = int('100111111000', 2)
        print('type_mask', self.sp_raw.type_mask)
        # LOCAL_NED: Inertial frame
        self.sp_raw.coordinate_frame = 1
        
        # initial values for setpoints
        self.sp_raw.position.x = 0.0
        self.sp_raw.position.y = 0.0
        self.sp_raw.position.z = 0.0

        # Yaw: Additional 90 because NED and XYZ is rotated 90 degrees in the horizontal plane (align N and X)
        self.sp_raw.yaw = math.radians(90)

    # Callbacks.

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

def publish_setpoint(cnt, pub_pos):
    pub_pos.publish(cnt.sp_raw)

def print_waypoint_update(current_wp, waypoints):
    print("Executing waypoint %d / %d. " % (current_wp + 1, len(waypoints)), "[N, E, D, Yaw] = ", waypoints[current_wp])

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
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Subscribe to drone's linear velocity
    rospy.Subscriber('mavros/local_position/velocity', TwistStamped, cnt.velCb)

    # Setpoint publishers
    if publish_to_mavros: # Either publish directly to mavros, or publish to local topic for other node to use
        print("Publishing topic: mavros/setpoint_raw/local (MAVROS)")
        sp_pos_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    else:
        print("Publishing topic: /setpoint_raw/local (NOT MAVROS)")
        sp_pos_pub = rospy.Publisher('setpoint_raw/local', PositionTarget, queue_size=1)

    # ROS main loop
    while not rospy.is_shutdown():
        cnt.sp_raw.acceleration_or_force.z = 0
        publish_setpoint(cnt, sp_pos_pub)
        rate.sleep()

def main(argv):
    try:
        run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
