#!/usr/bin/env python2
# ROS python API
import rospy

# import needed geometry messages
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped
from std_msgs.msg import Bool
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# import quat and eul transformation
from tf.transformations import euler_from_quaternion

# import other system/utils
import time, sys, math
import numpy

# Global variables

relative = False # Should the waypoints be relative from the initial position (except yaw)?
threshold = 0.2 # How small the error should be before sending the next waypoint
use_threshold = 0 # If True, wait to reach threshold before sending next waypoint, else use waypoint time

publish_to_mavros = 1 # If True, publish setpoints to mavros, else, use a different topic for other nodes to use
auto_arm = 1 # If True, automatically arm the drone, Else don't arm the drone
auto_takeoff_hold = 1 # If True, automatically switch to offboard mode, takeoff, then hold
auto_offboard = 1 # If True, automatically activate offboard mode before executing waypoints.
wait_for_offboard = 0 # If True, wait for manual switch to offboard mode before publishing waypoints
auto_land = 0 # If True, automatically lands after all waypoints
wait_for_simulink = 0 # Used when publish_to_mavros = 1; Wait for Simulink node to start before activating. Comment out Simulink publisher
waypoint_sequence = 2 # Choose which sequence of waypoints to use

# Waypoints = [N, E, D, Yaw (deg)]. D is entered as negative values
if waypoint_sequence == 1: # Spell 'ESL'
    waypoints = [
                    [0, 0, -2, 0],
                    [16, 0, -2, 0],
                    [16, 12, -2, 0],
                    [16, 0, -2, 0],
                    [8, 0, -2, 0],
                    [8, 8, -2, 0],
                    [8, 0, -2, 0],
                    [0, 0, -2, 0],

                    [0, 22, -2, 0],

                    [4, 26, -2, 0],
                    [8, 22, -2, 0],
                    [8, 18, -2, 0],
                    [12, 14, -2, 0],
                    [16, 18, -2, 0],

                    [16, 30, -2, 0],

                    [0, 30, -2, 0],
                    [0, 42, -2, 0],
                ]
    waypoints_time = numpy.ones((len(waypoints),1))*10 # Constant time interval for each waypoints
elif waypoint_sequence == 2: # X step
    waypoints = [
                    [0, 0, -2.5, 0],
                    [4.4, 0, -2.5, 0],
                    [4.4, 0, -2.5, 0],
                    [4.4, 0, -2.5, 0],
                ]
    waypoints_time = numpy.ones((len(waypoints),1))*5 # Constant time interval for each waypoints
else: # For system ID: Random waypoints and time intervals for system identification (from MATLAB script)
    waypoints = [
                    [0, 0, -10, 0],
                    [4.0736, 1.8116, -10, 0],
                    [1.3392, 3.7266, -11.9298, 0],
                    [5.3406, 4.0104, -12.7733, 0],
                    [5.5192, 5.7086, -14.6413, 0],
                    [2.2418, 5.3663, -13.2292, 0],
                    [6.3591, 3.9766, -12.595, 0],
                    [2.5315, 2.3862, -12.9687, 0],
                    [6.3049, 1.8341, -11.6093, 0],
                    [1.5062, 2.5149, -10.4388, 0],
                    [5.0016, 0.73311, -10, 0],
                    [0.798, 1.2417, -10, 0],
                    [-0.45742, 2.4738, -10.9466, 0],
                    [4.1285, 1.9021, -12.461, 0],
                    [4.3983, 0.84049, -14.0193, 0],
                    [4.4578, 0.16625, -14.3437, 0],
                    [7.4677, 0.69219, -13.0355, 0],
                    [8.6126, 2.5189, -13.3403, 0],
                    [10.826, 2.7322, -11.4165, 0],
                    [12.1253, 1.132, -10.5537, 0],
                    [12.8057, -0.60655, -11.7131, 0],
                    [11.0509, -1.633, -10.9094, 0],
                    [12.2507, -0.79852, -10.8101, 0],
                    [10.5621, 1.0016, -10.0716, 0],
                    [12.5817, 1.1945, -10.3356, 0],
                    [11.4078, 0.48818, -10, 0],
                    [15, -0.80731, -10.9018, 0],
                    [15, -0.44029, -10.1649, 0],
                    [11.1214, -1.4139, -11.0366, 0],
                    [7.0333, 0.17579, -12.3252, 0],
                    [11.7283, 1.9277, -11.2249, 0],
                    [9.3737, 1.4667, -10, 0],
                    [7.1952, 0.8445, -11.8468, 0],
                    [5.0008, 1.0667, -11.3306, 0],
                    [1.4448, 0.62324, -11.0958, 0],
                    [1.0172, 1.1482, -12.6978, 0],
                    [3.9098, 1.6228, -13.6155, 0],
                    [1.4653, 0.37466, -14.9738, 0],
                    [-2.9605, -1.4519, -13.3814, 0],
                    [-2.2777, -0.0094629, -13.5949, 0],
                    [-6.7324, -0.67779, -14.9924, 0],
                    [-4.3328, 1.1317, -16.2122, 0],
                    [-5.2474, 0.65179, -14.4391, 0],
                    [-8.8108, 1.6527, -13.497, 0],
                    [-6.2026, 1.8462, -11.8607, 0],
                    [-3.6096, -0.099754, -10.5627, 0],
                    [-4.027, 0.16659, -10.9095, 0],
                    [-2.0307, 1.2203, -10.0759, 0],
                    [-2.1081, -0.74779, -10, 0],
                    [-3.8056, 1.1555, -10, 0],
                    [-1.0662, -0.73, -10.8355, 0],
                    [-3.7619, -2.1262, -12.1685, 0],
                    [-6.5679, -3.8899, -10.8302, 0],
                    [-7.3499, -5.601, -10, 0],
                    [-7.9529, -4.422, -10, 0],
                    [-4.8675, -3.8914, -10, 0],
                    [-4.3286, -5.704, -11.7593, 0],
                    [-6.4549, -6.3295, -11.4363, 0],
                    [-4.1003, -7.7214, -10.0366, 0],
                    [-1.446, -6.4125, -10.8518, 0],
                    [0.17974, -6.6237, -10, 0],
                    [0.94802, -7.1857, -10.8802, 0],
                    [-3.7701, -5.9103, -10, 0],
                    [-7.2458, -5.7743, -10.5096, 0],
                    [-3.3432, -7.125, -10.523, 0],
                    [-1.0309, -7.9737, -11.4448, 0],
                    [-1.2098, -8.3255, -10.0013, 0],
                    [-0.25104, -9.8023, -10.487, 0],
                    [1.1865, -9.6201, -10, 0],
                    [4.4245, -10.9781, -11.2716, 0],
                    [3.8276, -9.7635, -12.1718, 0],
                    [7.1376, -10.5958, -13.8557, 0],
                    [4.4339, -12.3357, -14.3853, 0],
                    [6.8312, -11.0571, -15.4747, 0],
                    [1.8627, -11.4944, -15.2631, 0],
                    [3.6918, -9.9674, -16.5189, 0],
                    [4.3862, -8.5749, -16.7065, 0],
                    [6.3534, -9.9177, -15.224, 0],
                    [5.0427, -10.0067, -16.7339, 0],
                    [8.7244, -10.7961, -15.3671, 0],
                    [10.846, -11.3366, -15.7612, 0],
                    [14.6915, -12.1302, -14.1441, 0],
                    [9.945, -11.4751, -15.4867, 0],
                    [14.2549, -9.4953, -14.4578, 0],
                    [15, -10.9927, -16.109, 0],
                    [14.4415, -11.2653, -14.7517, 0],
                    [14.7163, -9.5639, -15.8728, 0],
                    [15, -7.5861, -15.8738, 0],
                    [12.6024, -5.9834, -16.3295, 0],
                    [15, -7.1553, -16.823, 0],
                    [15, -5.3738, -18.7876, 0],
                    [14.9151, -5.1321, -17.0622, 0],
                    [11.7657, -5.1961, -15.8328, 0],
                    [11.1502, -5.6071, -16.1258, 0],
                    [13.8432, -6.9974, -15.1276, 0],
                    [9.5782, -5.2496, -15.6681, 0],
                    [8.5484, -7.1454, -15.504, 0],
                    [11.4169, -7.0413, -13.6416, 0],
                    [15, -5.0725, -11.9237, 0],
                    [14.3303, -5.1343, -10.0454, 0],
                ]  
    waypoints_time = [ # Send next waypoint after waypoint_time[index] passed.
                0,
                9,
                12,
                15,
                11,
                7,
                11,
                15,
                12,
                13,
                9,
                8,
                13,
                7,
                12,
                8,
                7,
                7,
                11,
                8,
                14,
                8,
                12,
                9,
                6,
                15,
                8,
                19,
                13,
                10,
                10,
                9,
                19,
                14,
                13,
                12,
                9,
                6,
                15,
                18,
                13,
                14,
                19,
                7,
                15,
                17,
                6,
                12,
                12,
                11,
                15,
                6,
                19,
                12,
                10,
                14,
                6,
                14,
                10,
                13,
                9,
                13,
                15,
                10,
                6,
                12,
                14,
                8,
                15,
                9,
                10,
                14,
                15,
                13,
                12,
                8,
                12,
                14,
                11,
                10,
                11,
                17,
                8,
                8,
                7,
                8,
                17,
                13,
                17,
                15,
                14,
                13,
                8,
                9,
                12,
                11,
                14,
                18,
                11,
                12,
        ]

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

    # Switch to HOLD mode
    def setHoldMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LOITER')

        except rospy.ServiceException as e:
            print("setHoldMode - service set_mode call failed: %s. Hold Mode could not be set."%e)

    # Switch to OFFBOARD mode
    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("setOffboardMode - service set_mode call failed: %s. Offboard Mode could not be set."%e)

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
        self.pos_sp = PositionTarget()
        # set the flag to control N, E, D and yaw
        # type mask: http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html
        # set 0 for each parameter you want to command from your setpoint
        # set 1 for each parameter you want to ignore in your setpoint object
        # [yaw_rate, yaw, acc_sp_is_force, az, ay, ax, vz, vy, vx, z, y, x]
        self.pos_sp.type_mask = int('100111111000', 2)
        # LOCAL_NED: Inertial frame
        self.pos_sp.coordinate_frame = 1
        # initial values for setpoints
        self.pos_sp.position.x = 0.0
        self.pos_sp.position.y = 0.0
        self.pos_sp.position.z = 0.0

        # Yaw: Additional 90 because NED and XYZ is rotated 90 degrees in the horizontal plane (align N and X)
        self.pos_sp.yaw = math.radians(90)

        self.simulink_started = 0 # Bool whether simulink has started or not

    # Update setpoint message
    def updateSp(self, n_step, e_step, d_step, yaw_step):
        # Set step value (align N and X)
        self.pos_sp.position.y = n_step
        self.pos_sp.position.x = e_step
        self.pos_sp.position.z = -d_step

        # Yaw: Additional 90 because NED and XYZ is rotated 90 degrees in the horizontal plane (align N and X)
        self.pos_sp.yaw = math.radians(90 + yaw_step)

        # Set mask to control N, E, D and yaw
        self.pos_sp.type_mask = int('100111111000', 2)

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
    
    # Callback to check if simulink started
    def simulink_started_Cb(self, msg):
        self.simulink_started = msg.data

def publish_setpoint(cnt, pub_pos):
    pub_pos.publish(cnt.pos_sp)

def print_waypoint_update(current_wp, waypoints):
    print("Executing waypoint %d / %d. " % (current_wp + 1, len(waypoints)), "[N, E, D, Yaw] = ", waypoints[current_wp], "Time interval = ", waypoints_time[current_wp])

def activate_offboard_mode(cnt, modes, rate, pos_sp_pub):
    print("Activating OFFBOARD mode...")
    while not (cnt.state.mode == "OFFBOARD" or rospy.is_shutdown()):
        # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
        # PX4 will not activate OFFBOARD until setpoints are received from MAVROS
        k = 0
        while k<10:
            cnt.updateSp(cnt.local_pos.y, cnt.local_pos.x, -cnt.local_pos.z, cnt.local_yaw)
            publish_setpoint(cnt, pos_sp_pub)
            rate.sleep()
            k = k + 1

        modes.setOffboardMode()
        rate.sleep()
    print("OFFBOARD mode activated")

def activate_hold_mode(cnt, modes, rate, mavros_pos_sp_pub):
    print("Activating HOLD mode...")
    while not (cnt.state.mode == "AUTO.LOITER" or rospy.is_shutdown()):
        modes.setHoldMode()
        rate.sleep()
    print("HOLD mode activated")


def run(argv):
    # initiate node
    rospy.init_node('waypoint_scheduler_node')

    modes = FlightModes()
    cnt = Controller()
    rate = rospy.Rate(20.0) # ROS loop rate

    # Subscribers
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, cnt.velCb)
    rospy.Subscriber('simulink_started', Bool, cnt.simulink_started_Cb) # Custom topic to wait for simulink to start first

    # Publishers
    mavros_pos_sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10) # Published to MAVROS to control uav
    transfer_pos_sp_pub = rospy.Publisher('setpoint_raw/local', PositionTarget, queue_size=10) # Topic used by other nodes, NOT mavros
    
    # Save initial position
    init_pos = Point(cnt.local_pos.x, cnt.local_pos.y, cnt.local_pos.z)
    if not relative:
        init_pos = Point(0, 0, 0)

    # Arm the drone
    if auto_arm:
        print("Arming...")
        while not (cnt.state.armed or rospy.is_shutdown()):
            modes.setArm()
            rate.sleep()
        print("Armed")
        print("")

    # Takeoff and Hold
    if auto_takeoff_hold and (abs(cnt.local_pos.z) <= 0.3):
        print("Starting takeoff and hold...")
        activate_offboard_mode(cnt, modes, rate, mavros_pos_sp_pub)
        print("Takeoff...")
        while (not rospy.is_shutdown()): # Run until takeoff reached breaks out of while loop
            y = init_pos.y
            x = init_pos.x
            z = init_pos.z + 2.5
            yaw = waypoints[0][3]

            cnt.updateSp(y, x, -z, yaw) # Note conversion from NED to ENU for mavros (swop x and y. invert sign of z). Publish in ENU frame
            publish_setpoint(cnt, mavros_pos_sp_pub)

            if targetReached(x, cnt.local_pos.x, threshold) and targetReached(0, cnt.local_vel.x, threshold) and \
               targetReached(y, cnt.local_pos.y, threshold) and targetReached(0, cnt.local_vel.y, threshold) and \
               targetReached(z, cnt.local_pos.z, threshold) and targetReached(0, cnt.local_vel.z, threshold) and \
               targetReached(yaw, cnt.local_yaw, threshold):
                
                print("Takeoff complete.")
                activate_hold_mode(cnt, modes, rate, mavros_pos_sp_pub)
                
                break

            rate.sleep()
        print("")

    # Activate OFFBOARD mode
    if auto_offboard:
        print("Starting auto-offboard process...")
        if publish_to_mavros: 
            if wait_for_simulink:
                print("Waiting for Simulink node, but use onboard control...")
                while not cnt.simulink_started and not rospy.is_shutdown(): # Wait for Simulink before activating
                    rospy.Rate(20.0).sleep  
            else:
                print("Waiting for other node to publish MAVROS setpoints...")  
            activate_offboard_mode(cnt, modes, rate, mavros_pos_sp_pub)

        else: # Wait for other node to publish setpoints before offboard will activate
            activate_offboard_mode(cnt, modes, rate, transfer_pos_sp_pub)

        print("")

    # Wait for manual switch to OFFBOARD mode before proceding
    elif wait_for_offboard:
        print("Waiting for user to switch to OFFBOARD mode on QGC...")
        while (cnt.state.mode != "OFFBOARD") and not rospy.is_shutdown():
            rate.sleep()
        print("User activated OFFBOARD mode")
        print("")

    # ROS main loop
    current_wp = 0
    last_time = rospy.Time.now().to_sec()
    print("Start publishing waypoints...")

    if publish_to_mavros:
        print("Publishing setpoint topic: mavros/setpoint_raw/local (MAVROS)")
    else:
        print("Publishing setpoint topic: /setpoint_raw/local (NOT MAVROS)")

    print_waypoint_update(current_wp, waypoints)

    while current_wp < len(waypoints) and not rospy.is_shutdown():
        y = init_pos.y + waypoints[current_wp][0]
        x = init_pos.x + waypoints[current_wp][1]
        z = init_pos.z + waypoints[current_wp][2]
        yaw = waypoints[current_wp][3]

        cnt.updateSp(y, x, z, yaw)
        if publish_to_mavros: # Either publish directly to mavros, or publish to local topic to transfer to other node to use
            publish_setpoint(cnt, mavros_pos_sp_pub)
            publish_setpoint(cnt, transfer_pos_sp_pub) # Also publish to other topic for other nodes to see
        else:
            publish_setpoint(cnt, transfer_pos_sp_pub)
        
        rate.sleep()

        # print("X:%.3f - Y:%.3f - Z:%.3f" % ( (x - cnt.local_pos.x), (y - cnt.local_pos.y), (z - cnt.local_pos.z) ))

        if use_threshold:
            if targetReached(x, cnt.local_pos.x, threshold) and targetReached(0, cnt.local_vel.x, threshold) and \
               targetReached(y, cnt.local_pos.y, threshold) and targetReached(0, cnt.local_vel.y, threshold) and \
               targetReached(z, cnt.local_pos.z, threshold) and targetReached(0, cnt.local_vel.z, threshold) and \
               targetReached(yaw, cnt.local_yaw, threshold):

                current_wp = current_wp + 1
                if current_wp < len(waypoints):
                    print_waypoint_update(current_wp, waypoints)
        else:
            current_time = rospy.Time.now().to_sec()
            if current_time - last_time >= waypoints_time[current_wp]:
                current_wp = current_wp + 1
                if current_wp < len(waypoints):
                    print_waypoint_update(current_wp, waypoints)
                last_time = current_time
    print("Last waypoint reached")
    print("")

    if auto_land:
        print("Landing")
        while not (cnt.state.mode == "AUTO.LAND" or rospy.is_shutdown()):
            modes.setLandMode()
            rate.sleep()
        print("Landed")
        print("")
    else:
        # Stay at last waypoint until OFFBOARD mode is terminated
        print("Waiting for pilot to switch out of OFFBOARD mode...")
        while (cnt.state.mode == "OFFBOARD" or not rospy.is_shutdown()):
            cnt.updateSp(init_pos.y + waypoints[current_wp - 1][0], init_pos.x + waypoints[current_wp - 1][1], -init_pos.z - waypoints[current_wp - 1][2], -waypoints[current_wp - 1][3])
            publish_setpoint(cnt, mavros_pos_sp_pub)
            rate.sleep()
    print("Done")
    print("")

def targetReached(setpoint, current, threshold):
    return abs(current - setpoint) < threshold

def main(argv):
    try:
        run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Terminated\n")

if __name__ == "__main__":
    main(sys.argv[1:])
