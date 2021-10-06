/**
 * @file optflow_to_vector3.cpp
 * @brief Converts a OpticalFlow message to a Vector3 message, because mavros messages are not working in Simulink.
 * Input: 
 * - OpticalFlow message from MAVROS which contains the payload angles
 * Output: 
 * - Vector3 message of payload euler angles so Simulink can access it
 */

#define _USE_MATH_DEFINES
 
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/OpticalFlowRad.h>

inline double deg2rad(double degrees) {
    return degrees * (M_PI / 180.0);
}

// Subscriber callback functions and variables
geometry_msgs::Vector3 payload_angles; // Payload Euler angles extracted from optical flow
void optical_flow_callback(const mavros_msgs::OpticalFlowRad::ConstPtr& msg){
    payload_angles.x = (*msg).integrated_xgyro;
    payload_angles.y = (*msg).integrated_ygyro;
    payload_angles.z = (*msg).integrated_zgyro;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optflow_to_vector3");
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber optical_flow_sub = nh.subscribe<mavros_msgs::OpticalFlowRad>
            ("/mavros/px4flow/raw/optical_flow_rad", 10, optical_flow_callback); // Received vector3 from Simulink ROS node

    // Publisher
    ros::Publisher payload_angles_pub = nh.advertise<geometry_msgs::Vector3>
            ("/simulink/payload_angles", 10);

    // Frequency of Node
    float node_freq = 100;
    ros::Rate rate(node_freq);

    ROS_INFO("Starting optflow_to_vector3 node...");
    while(ros::ok()){   

        // Publish angles:    
        payload_angles_pub.publish(payload_angles); // Publish to MAVROS

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}