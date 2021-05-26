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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float64.h>

inline double deg2rad(double degrees) {
    return degrees * (M_PI / 180.0);
}

// Subscriber callback functions and variables
mavros_msgs::PositionTarget received_setpoint; // Received setpoint from waypoints_scheduler.py
void setpoint_callback(const mavros_msgs::PositionTarget::ConstPtr& msg){
    received_setpoint = *msg;
    // ROS_INFO("setpoint_raw: [x: %f, y: %f, z: %f, yaw: %f]", (*msg).position.x, (*msg).position.y, (*msg).position.z, (*msg).yaw);
}
geometry_msgs::PoseStamped local_position;
void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
    // ROS_INFO("Local position: [x: %f, y: %f, z: %f]", (*msg).pose.position.x, (*msg).pose.position.y, (*msg).pose.position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pos_control_node");
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    // Subscribers
    ros::Subscriber received_setpoint_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("/setpoint_raw/local", 10, setpoint_callback); // Received setpoint from mpc_waypoints
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_callback);

    // Initialise publisher messages:
    mavros_msgs::PositionTarget setpoint_raw;
    setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw.type_mask =    mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                // mavros_msgs::PositionTarget::IGNORE_VX |
                                // mavros_msgs::PositionTarget::IGNORE_VY |
                                // mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                // mavros_msgs::PositionTarget::IGNORE_YAW |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE; // You command each parameter not ignored

    // setpoint_raw.header.frame_id = "map";
    setpoint_raw.header.stamp = ros::Time::now();

    // Frequency of Node
    ros::Rate rate(50.0);
    
    ROS_INFO("Started offboard position control");
    while(ros::ok()){        

        // Get params ??? Add code to get this from FCU
        double mpc_xy_p = 0.95;
        double mpc_z_p  = 1.0;

        // Position controller

        // Hardcode input
        // received_setpoint.position.x = 0;
        // received_setpoint.position.y = 0;
        // received_setpoint.position.z = 5;
        // received_setpoint.yaw = deg2rad(90);

        vel_sp_x = (received_setpoint.position.x - local_position.pose.position.x)*mpc_xy_p;
        vel_sp_y = (received_setpoint.position.y - local_position.pose.position.y)*mpc_xy_p;
        vel_sp_z = (received_setpoint.position.z - local_position.pose.position.z)*mpc_z_p;        
        
        // ROS_INFO("setpoint_raw mavros: [vx: %f, vy: %f, vz: %f, yaw: %f]", setpoint_raw.velocity.x, setpoint_raw.velocity.y, setpoint_raw.velocity.z, setpoint_raw.yaw);
        // ROS_INFO("vel_sp X: %f, Y: %f, Z: %f", setpoint_raw.velocity.x, setpoint_raw.velocity.y, setpoint_raw.velocity.z);
        // ??? add constraints like in PX4

        // Publish setpoint to MAVROS:
        setpoint_raw.yaw = received_setpoint.yaw;
        setpoint_raw_pub.publish(setpoint_raw);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}