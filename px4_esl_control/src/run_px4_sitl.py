#!/usr/bin/env python3

# ROS python API
import rospy


# import other system/utils
import subprocess
import time, sys, math
import os

def run(argv):
    # initiate node
    rospy.init_node('run_px4_sitl')

    completed = subprocess.run('cd ~/Masters/Developer/PX4-Autopilot/', shell=True)

        #     shell=True,
        #     stdout=subprocess.PIPE,
        # )
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep() # Wait a bit, otherwise first row of log is zeros

def main(argv):
    try:
        run(argv)
    except rospy.ROSInterruptException:
        pass

    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
