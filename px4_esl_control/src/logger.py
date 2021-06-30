#!/usr/bin/env python2

## Log SITL data and write to CSV file

## Need this node to gather payload angle and uav data with synced timestamp
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

def append_list_as_row(file_name, list_of_elements):
    # Open file in append mode
    with open(file_name, 'a+') as write_obj:
        # Create a writer object from csv module
        csv_writer = writer(write_obj)
        # Add contents of list as last row in the csv file
        csv_writer.writerow(list_of_elements)

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
        # Link states has arrays where index [0,1,2,3] = [ground_plane, base_link, payload, imu_link]
        # self.uav_quat = msg.pose[1].orientation # UAV quaternion
        self.payload_quat = msg.pose[2].orientation # Payload quaternion
        # print(self.uav_quat.x, self.uav_quat.y, self.uav_quat.z, self.uav_quat.w)
    
    def velocity_cb(self, msg): # Callback function for local velocity subscriber
        # Convert from ENU (MAVROS) to NED frame
        self.velocity.x = msg.twist.linear.y
        self.velocity.y = msg.twist.linear.x
        self.velocity.z = -msg.twist.linear.z
        # print_Vector3(self.velocity)
        
    
    def acc_sp_cb(self, msg):  # Callback function for acceleration setpoint subscriber
        # Convert from ENU (MAVROS) to NED frame
        self.acc_sp.x = msg.acceleration_or_force.y
        self.acc_sp.y = msg.acceleration_or_force.x
        self.acc_sp.z = -msg.acceleration_or_force.z
        # print_Vector3(self.acc_sp) 

    def pos_sp_cb(self, msg): # Callback function for position setpoint subscriber
        # Convert from ENU (MAVROS) to NED frame
        self.pos_sp.x = msg.position.y
        self.pos_sp.y = msg.position.x
        self.pos_sp.z = -msg.position.z
        # print_Vector3(self.pos_sp) 


def run(argv):
    # initiate node
    rospy.init_node('logger_node')

    # ROS loop rate
    rate = rospy.Rate(100.0) # Logging frequency Hz

    # Object for subscribing
    sub = Sub()

    # Subscribers
    rospy.Subscriber('gazebo/link_states', LinkStates, sub.link_states_cb) # For payload angles
    rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, sub.velocity_cb) # For velocity
    rospy.Subscriber('mavros/setpoint_raw/target_local', PositionTarget, sub.acc_sp_cb) # For acc_sp
    rospy.Subscriber('setpoint_raw/local', PositionTarget, sub.pos_sp_cb) # For pos_sp

    # Log file
    parent_folder = "/home/esl/Masters/Developer/MATLAB/Quad_Sim_Murray/system_id/SITL/honeybee_payload/data"
    date_folder = datetime.now().strftime("%Y-%m-%d")
    log_path = os.path.join(parent_folder, date_folder)
    
    try: 
        os.mkdir(log_path) 
    except OSError as error: 
        print(error)
    
    file_time = datetime.now().strftime("custom_log_%H_%M_%S.csv")
    file_name = os.path.join(log_path, file_time)    

    print("Log file:",file_name)

    column_headings = ['current_time',    'velocity.x', 'velocity.y', 'velocity.z',    'acc_sp.x', 'acc_sp.y', 'acc_sp.y',    'payload_angles.x', 'payload_angles.y',    'pos_sp.x', 'pos_sp.y', 'pos_sp.z']
    append_list_as_row(file_name, column_headings)

    rospy.Rate(1).sleep() # Wait a bit, otherwise first row of log is zeros

    print("")
    print("Start logging...")

    while not rospy.is_shutdown():
        
        # Get all variables at once
        current_time    = rospy.Time.now().to_sec()
        # uav_quat        = sub.uav_quat
        payload_quat    = sub.payload_quat
        velocity        = sub.velocity
        acc_sp          = sub.acc_sp
        pos_sp          = sub.pos_sp

        # Convert quat to angles
        payload_angles_tuple = euler_from_quaternion([payload_quat.x, payload_quat.y, payload_quat.z, payload_quat.w], 'sxyz') # Gazebo frame (Drone N = x, E = -y, D = -z)
        
        # Convert to NED local frame (angles are relative to Drone heading)
        payload_angles.x = payload_angles_tuple[0]
        payload_angles.y = -payload_angles_tuple[1]
        payload_angles.z = -payload_angles_tuple[2]

        # Log data for system identification. Print to csv file
        if log_data:       
            new_row = [current_time,   velocity.x, velocity.y, velocity.z,    acc_sp.x, acc_sp.y, acc_sp.z,    payload_angles.x, payload_angles.y,    pos_sp.x, pos_sp.y, pos_sp.z] # New row to append to log file
            append_list_as_row(file_name, new_row)

        time_passed = rospy.Time.now().to_sec() - current_time 
        if time_passed >= 0.02:
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
