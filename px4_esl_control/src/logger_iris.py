#!/usr/bin/env python2

## Log HIL data for system ID and write to CSV file

## Need this node to gather payload angle and uav data with synced timestamp

# ROS python API
import rospy

# import messages and services
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped, Quaternion
from mavros_msgs.msg import PositionTarget, OpticalFlowRad

# import quat and eul transformation
from tf.transformations import euler_from_quaternion

# import other system/utils
import time, sys, math
from csv import writer
from datetime import datetime
import os
import getpass

# Global variables
username = getpass.getuser()
payload_angles = Vector3()  # x,y,z angles of payload
write_obj = open('temp.csv', 'a+') # Temp write object so that it can run close() when terminated

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
        self.position = Vector3()
        self.velocity = Vector3()
        self.pos_sp = Vector3()
        self.vel_sp = Vector3()
        self.acc_sp = Vector3()
        self.payload_euler = Vector3()
        # self.payload_quat = Quaternion()
        # self.uav_quat = Quaternion()
        self.simulink_acc_sp = Vector3()
        self.last_pub_time = rospy.Time.now().to_nsec()

    def payload_angle_cb(self, msg): # Callback function for payload angle (Euler)
        # Already in NED frame
        # Payload Euler angles hidden in optical flow messages
        self.payload_euler.x = msg.integrated_xgyro
        self.payload_euler.y = msg.integrated_ygyro
        self.payload_euler.z = msg.integrated_zgyro
        # print_Vector3(self.payload_euler)

    def position_cb(self, msg): # Callback function for local position subscriber
        # Convert from ENU (MAVROS) to NED frame
        self.position.x = msg.pose.position.y
        self.position.y = msg.pose.position.x
        self.position.z = -msg.pose.position.z
        # print_Vector3(self.position)
 
    def velocity_cb(self, msg): # Callback function for local velocity subscriber
        # Convert from ENU (MAVROS) to NED frame
        self.velocity.x = msg.twist.linear.y
        self.velocity.y = msg.twist.linear.x
        self.velocity.z = -msg.twist.linear.z
        # print_Vector3(self.velocity)

    def pos_sp_cb(self, msg): # Callback function for position setpoint subscriber
        # Convert from ENU (MAVROS) to NED frame
        self.pos_sp.x = msg.position.y
        self.pos_sp.y = msg.position.x
        self.pos_sp.z = -msg.position.z
        # print_Vector3(self.pos_sp)        
    
    def raw_sp_cb(self, msg):  # Callback function for acceleration and velocity setpoint subscriber
        # Convert from ENU (MAVROS) to NED frame
        self.vel_sp.x = msg.velocity.y
        self.vel_sp.y = msg.velocity.x
        self.vel_sp.z = -msg.velocity.z
        # print_Vector3(self.vel_sp)    

        self.acc_sp.x = msg.acceleration_or_force.y
        self.acc_sp.y = msg.acceleration_or_force.x
        self.acc_sp.z = -msg.acceleration_or_force.z
        # print_Vector3(self.acc_sp) s

    def simulink_acc_sp_cb(self, msg): # Callback for acc_sp sent from Simulink node
        self.simulink_acc_sp.x = msg.x
        self.simulink_acc_sp.y = msg.y
        self.simulink_acc_sp.z = msg.z
        
def run(argv):
    # initiate node
    rospy.init_node('logger_node')

    # ROS loop rate
    rate = rospy.Rate(100.0) # Logging frequency Hz

    # Object for subscribing
    sub = Sub()

    # Subscribers
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, sub.position_cb)
    rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, sub.velocity_cb) # For velocity
    rospy.Subscriber('mavros/setpoint_raw/target_local', PositionTarget, sub.raw_sp_cb) # For acc_sp
    rospy.Subscriber('mavros/px4flow/raw/optical_flow_rad', OpticalFlowRad, sub.payload_angle_cb) # For acc_sp
    rospy.Subscriber('setpoint_raw/local', PositionTarget, sub.pos_sp_cb) # For pos_sp
    rospy.Subscriber('simulink/acc_sp', Vector3, sub.simulink_acc_sp_cb) # For pos_sp

    # Log file
    if username == 'honeybee':
        parent_folder = "/home/honeybee/sys_id/data" # Folder on Jetson Nano
    elif username == 'murray':
        parent_folder = "/home/murray/Masters/Developer/MATLAB/Quad_Sim_Murray/system_id/HITL/iris/data"
    
    date_folder = datetime.now().strftime("%Y-%m-%d")
    log_path = os.path.join(parent_folder, date_folder)
    
    try: 
        os.mkdir(log_path) 
    except OSError as error: 
        print(error)
    
    file_time = datetime.now().strftime("custom_log_%H_%M_%S.csv")
    file_name = os.path.join(log_path, file_time)
    os.remove('temp.csv') # Delete temp file   
    write_obj = open(file_name, 'a+') # Object to write with append mode   
    csv_writer = writer(write_obj) # Create a writer object from csv module
    
    print("Log file:",file_name)

    # Only append to this list. Other scripts depend on order of columns, not heading names
    column_headings = [ 'current_time',
                        'position.x', 'position.y', 'position.z',
                        'velocity.x', 'velocity.y', 'velocity.z',
                        'pos_sp.x', 'pos_sp.y', 'pos_sp.z',
                        'vel_sp.x', 'vel_sp.y', 'vel_sp.z', 
                        'acc_sp.x', 'acc_sp.y', 'acc_sp.y',
                        'payload_euler.x', 'payload_euler.y', 'payload_euler.z',
                        'simulink_acc_sp.x', 'simulink_acc_sp.y', 'simulink_acc_sp.y',
                    ]
  
    csv_writer.writerow(column_headings) # Add contents of list as last row in the csv file
    # append_list_as_row(file_name, column_headings)

    rospy.Rate(1).sleep() # Wait a bit, otherwise first row of log is zeros

    print("")
    print("Start logging...")

    while not rospy.is_shutdown():
        
        # Get all variables at once to minimise difference in timing after calculations
        current_time    = rospy.Time.now().to_sec()
        position        = sub.position
        velocity        = sub.velocity
        pos_sp          = sub.pos_sp
        vel_sp          = sub.vel_sp
        acc_sp          = sub.acc_sp        
        payload_euler   = sub.payload_euler
        simulink_acc_sp = sub.simulink_acc_sp
        
        # # Convert quat to angles
        # payload_angles_tuple = euler_from_quaternion([payload_quat.x, payload_quat.y, payload_quat.z, payload_quat.w], 'sxyz') # Gazebo frame (Drone N = x, E = -y, D = -z)
        # # Convert to NED local frame (angles are relative to Drone heading)
        # payload_angles.x = payload_angles_tuple[0]
        # payload_angles.y = -payload_angles_tuple[1]
        # payload_angles.z = -payload_angles_tuple[2]

        # Log data for system identification. Print to csv file     
        new_row = [ current_time,
                    position.x, position.y, position.z,
                    velocity.x, velocity.y, velocity.z,
                    pos_sp.x, pos_sp.y, pos_sp.z,
                    vel_sp.x, vel_sp.y, vel_sp.z, 
                    acc_sp.x, acc_sp.y, acc_sp.y,
                    payload_euler.x, payload_euler.y, payload_euler.z,
                    simulink_acc_sp.x, simulink_acc_sp.y, simulink_acc_sp.z,
                ]

        csv_writer.writerow(new_row) # Add contents of list as last row in the csv file
        # append_list_as_row(file_name, new_row)

        time_passed = rospy.Time.now().to_sec() - current_time 
        if time_passed >= 0.02:
            print(time_passed)
        
        rate.sleep()

def main(argv):
    try:
        run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Closing write object...\n")
    write_obj.close()

    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
