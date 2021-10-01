#!/usr/bin/env python2

## Subscribe to /gazebo/link_states topic, convert quaternion and publish payload angles
## Need to start PX4 and gazebo in seperate terminals to start gazebo with ROS wrapper:

# Terminal 1:
# no_sim=1 make px4_sitl_default gazebo_honeybee_payload

# Terminal 2:
# source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
# roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/honeybee_payload.world

# Have to include honeybee model in honeybee.world

# ROS python API
import rospy

# import messages and services
from std_msgs.msg import Int64
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped, Quaternion
from mavros_msgs.msg import PositionTarget
from gazebo_msgs.msg import LinkStates

# import quat and eul transformation
from tf.transformations import euler_from_quaternion

# import other system/utils
import time, sys, math
from csv import writer
from datetime import datetime
import os

def print_Vector3(vector):
    print("X:%.3f - Y:%.3f - Z:%.3f" % (vector.x, vector.y, vector.z))

# Class for subscribing
class Sub: 
    def __init__(self):
        # Local variables
        # self.uav_quat = Quaternion()
        self.payload_quat = Quaternion()
        self.payload_angles_rate = Vector3()

    def link_states_cb(self, msg): # Callback function for link_states subscriber
        # Link states has arrays where each index is a different sdf link
            # 1 - iris::base_link
            # 2 - iris::/imu_link
            # 3 - iris::rotor_0
            # 4 - iris::rotor_1
            # 5 - iris::rotor_2
            # 6 - iris::rotor_3
            # 7 - iris::payload
        self.payload_quat = msg.pose[7].orientation # Payload quaternion

def run(argv):
    # initiate node
    rospy.init_node('payload_angle_node')

    # ROS loop rate
    rate = rospy.Rate(100.0) # Publishing frequency

    # Object for subscribing
    sub = Sub()

    # Initialise
    payload_angles = Vector3()  # x,y,z angles of payload

    # Subscribers
    rospy.Subscriber('gazebo/link_states', LinkStates, sub.link_states_cb) # For payload angles

    # Publishers
    payload_angles_pub      = rospy.Publisher('payload_angles_local', Vector3, queue_size=10) # Publish angles in local NED frame (i.e. only relative to uav yaw)
    
    # payload_angles_rate_pub = rospy.Publisher('payload_angles_rate_local', Vector3, queue_size=10) # Publish angle rates in local NED frame (i.e. only relative to uav yaw)
    
    print("")
    print("Publishing angles...")

    while not rospy.is_shutdown():
        
        payload_quat    = sub.payload_quat

        # Convert quat to angles
        payload_angles_tuple = euler_from_quaternion([payload_quat.x, payload_quat.y, payload_quat.z, payload_quat.w], 'sxyz') # Gazebo frame (Drone N = x, E = -y, D = -z)

        # Convert to NED local frame (angles are relative to Drone heading)
        payload_angles.x = payload_angles_tuple[0]
        payload_angles.y = -payload_angles_tuple[1]
        payload_angles.z = -payload_angles_tuple[2]    

        # Publish to ROS topic
        payload_angles_pub.publish(payload_angles)

        rate.sleep()

def main(argv):
    try:
        run(argv)
    except rospy.ROSInterruptException:
        pass
    print("")
    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
