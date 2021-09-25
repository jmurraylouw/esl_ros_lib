/**
 * @file pos_control_node.cpp
 * @brief Position control node for offboard control of a quarotor running PX4.
 * Input: 
 * - Local_NED position (MAVROS topic: mavros/local_position/pose)
 * - Local_NED position setpoint (ROS topic: /setpoint_raw/local (not in mavros))
 * Output: 
 * - Local_NED velocity setpoint (ROS topic: /setpoint_raw/local (not in mavros))
 */

#define _USE_MATH_DEFINES
 
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/PositionTarget.h>

inline double deg2rad(double degrees) {
    return degrees * (M_PI / 180.0);
}

// Subscriber callback functions and variables
geometry_msgs::Vector3 vector3; // Received vector3 from Simulink ROS node
void vector3_callback(const geometry_msgs::Vector3::ConstPtr& msg){
    vector3 = *msg;
    // ROS_INFO("setpoint_raw: [x: %f, y: %f, z: %f, yaw: %f]", (*msg).x, (*msg).y, (*msg).z;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pos_control_node");
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    // Subscribers
    ros::Subscriber vector3_sub = nh.subscribe<geometry_msgs::Vector3>
            ("/simulink/pos_sp", 10, vector3_callback); // Received vector3 from Simulink ROS node
    
    // Initialise publisher messages:
    mavros_msgs::PositionTarget setpoint_raw;
    setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw.type_mask =    
                                // mavros_msgs::PositionTarget::IGNORE_PX |
                                // mavros_msgs::PositionTarget::IGNORE_PY |
                                // mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                // mavros_msgs::PositionTarget::IGNORE_YAW |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE; // You command each parameter that is commented out (not ignored)
    ROS_INFO("setpoint_raw, type_mask: %d", setpoint_raw.type_mask);

    // Frequency of Node
    float node_freq = 30.0;
    ros::Rate rate(node_freq);

    ROS_INFO("Starting position control node...");
    while(ros::ok()){   

        // Receive from simulink
        setpoint_raw.position.x = vector3.x;
        setpoint_raw.position.y = vector3.y;
        setpoint_raw.position.z = vector3.z;

        // Set Yaw = North direction
        setpoint_raw.yaw = deg2rad(90);

        // Publish setpoint:    
        setpoint_raw_pub.publish(setpoint_raw); // Publish to MAVROS

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}