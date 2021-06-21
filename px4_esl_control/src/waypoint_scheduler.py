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

# Global variables

relative = False # Should the waypoints be relative from the initial position (except yaw)?
threshold = 0.2 # How small the error should be before sending the next waypoint
waypoint_time = 40 # If waypoint_time < 0, send next waypoint after current one is within threshold. Else, send next waypoint after waypoint_time passed.

publish_to_mavros = 0 # If True, publish setpoints to mavros, else, use a different topic for other nodes to use
auto_arm = 1 # If True, automatically arm the drone, Else don't arm the drone
auto_takeoff_hold = 1 # If True, automatically switch to offboard mode, takeoff, then hold
auto_offboard = 1 # If True, automatically activate offboard mode before executing waypoints.
wait_for_offboard = 0 # If True, wait for manual switch to offboard mode before publishing waypoints
auto_land = 1 # If True, automatically lands after all waypoints

# Waypoints = [N, E, D, Yaw (deg)]. D is entered as postive values, but script converts it to negative
waypoints = [
                [0, 0, 2, 0],
                [16, 0, 2, 0],
                [16, 12, 2, 0],
                [16, 0, 2, 0],
                [8, 0, 2, 0],
                [8, 8, 2, 0],
                [8, 0, 2, 0],
                [0, 0, 2, 0],

                [0, 22, 2, 0],

                [4, 26, 2, 0],
                [8, 22, 2, 0],
                [8, 18, 2, 0],
                [12, 14, 2, 0],
                [16, 18, 2, 0],

                [16, 30, 2, 0],

                [0, 30, 2, 0],
                [0, 42, 2, 0],
            ]

waypoints = [
                [0, 0, 2.5, 0],
                [5, 0, 2.5, 0],
                [5, 5, 2.5, 0],
                [5, 5, 7.5, 0],
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

def publish_setpoint(cnt, pub_pos):
    pub_pos.publish(cnt.pos_sp)

def print_waypoint_update(current_wp, waypoints):
    print("Executing waypoint %d / %d. " % (current_wp + 1, len(waypoints)), "[N, E, D, Yaw] = ", waypoints[current_wp])

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
            activate_offboard_mode(cnt, modes, rate, mavros_pos_sp_pub)
        else:
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
    last_time = time.time()
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

        cnt.updateSp(y, x, -z, yaw)
        if publish_to_mavros: # Either publish directly to mavros, or publish to local topic to transfer to other node to use
            publish_setpoint(cnt, mavros_pos_sp_pub)
        else:
            publish_setpoint(cnt, transfer_pos_sp_pub)
        
        rate.sleep()

        print("X:%.3f - Y:%.3f - Z:%.3f" % ( (x - cnt.local_pos.x), (y - cnt.local_pos.y), (z - cnt.local_pos.z) ))

        if waypoint_time < 0:
            if targetReached(x, cnt.local_pos.x, threshold) and targetReached(0, cnt.local_vel.x, threshold) and \
               targetReached(y, cnt.local_pos.y, threshold) and targetReached(0, cnt.local_vel.y, threshold) and \
               targetReached(z, cnt.local_pos.z, threshold) and targetReached(0, cnt.local_vel.z, threshold) and \
               targetReached(yaw, cnt.local_yaw, threshold):

                current_wp = current_wp + 1
                if current_wp < len(waypoints):
                    print_waypoint_update(current_wp, waypoints)
        else:
            current_time = time.time()
            if current_time - last_time >= waypoint_time:
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
