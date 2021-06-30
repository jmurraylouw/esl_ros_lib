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

# Global variables
payload_angles = Vector3()  # x,y,z angles of payload
log_data = 1 # Log data to csv file or not

def print_Vector3(vector):
    print("X:%.3f - Y:%.3f - Z:%.3f" % (vector.x, vector.y, vector.z))

# Class for subscribing
class Sub: 
    def __init__(self):
        # Local variables
        # self.uav_quat = Quaternion()
        self.payload_quat = Quaternion()
        self.velocity = Vector3()
        self.acc_sp = Vector3()
        self.pos_sp = Vector3()

    def link_states_cb(self, msg): # Callback function for link_states subscriber
        # Lin
        # k states has arrays where index [0,1,2,3] = [ground_plane, base_link, payload, imu_link]
        # self.uav_quat = msg.pose[1].orientation # UAV quaternion
        self.payload_quat = msg.pose[2].orientation # Payload quaternion
        # print(self.uav_quat.x, self.uav_quat.y, self.uav_quat.z, self.uav_quat.w)

def run(argv):
    # initiate node
    rospy.init_node('payload_angle_node')

    # ROS loop rate
    rate = rospy.Rate(100.0) # Publishing frequency

    # Object for subscribing
    sub = Sub()

    # Subscribers
    rospy.Subscriber('gazebo/link_states', LinkStates, sub.link_states_cb) # For payload angles

    # Publishers
    payload_angles_pub = rospy.Publisher('payload_angles_local', Vector3, queue_size=10) # Publish angles in local NED frame (i.e. only relative to uav yaw)
    
    print("")
    print("Publishing angles...")

    while not rospy.is_shutdown():
        
        # Get all variables at once
        current_time    = rospy.Time.now().to_sec()

        # uav_quat        = sub.uav_quat
        payload_quat    = sub.payload_quat

        # Convert quat to angles
        payload_angles_tuple = euler_from_quaternion([payload_quat.x, payload_quat.y, payload_quat.z, payload_quat.w], 'sxyz') # Gazebo frame (Drone N = x, E = -y, D = -z)
        
        # Convert to NED local frame (angles are relative to Drone heading)
        payload_angles.x = payload_angles_tuple[0]
        payload_angles.y = -payload_angles_tuple[1]
        payload_angles.z = -payload_angles_tuple[2]

        # Publish angles to ROS topic
        payload_angles_pub.publish(payload_angles)

        # Check if there was a large delay
        time_passed = rospy.Time.now().to_sec() - current_time    
        if time_passed > 0.01:
            print(time_passed)

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
