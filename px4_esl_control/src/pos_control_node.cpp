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
#include <geometry_msgs/TwistStamped.h>
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
mavros_msgs::PositionTarget target_setpoint_raw; // Received setpoint from waypoints_scheduler.py
void target_setpoint_raw_callback(const mavros_msgs::PositionTarget::ConstPtr& msg){
    target_setpoint_raw = *msg;
    // ROS_INFO("setpoint_raw: [x: %f, y: %f, z: %f, yaw: %f]", (*msg).position.x, (*msg).position.y, (*msg).position.z, (*msg).yaw);
}
geometry_msgs::PoseStamped local_position;
void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
    // ROS_INFO("Local position: [x: %f, y: %f, z: %f]", (*msg).pose.position.x, (*msg).pose.position.y, (*msg).pose.position.z);
}
geometry_msgs::TwistStamped local_velocity;
void local_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_velocity = *msg;
    ROS_INFO("vel x: %f", (*msg).twist.linear.x);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pos_control_node");
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    ros::Publisher local_setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("setpoint_raw/local", 10);

    // Subscribers
    ros::Subscriber received_setpoint_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("/setpoint_raw/local", 10, setpoint_callback); // Received setpoint from mpc_waypoints
    ros::Subscriber target_setpoint_raw_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/target_local", 10, target_setpoint_raw_callback);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_callback);
    ros::Subscriber local_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity", 10, local_velocity_callback);


    // Initialise publisher messages:
    mavros_msgs::PositionTarget setpoint_raw;
    setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw.type_mask =    mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                // mavros_msgs::PositionTarget::IGNORE_AFX |
                                // mavros_msgs::PositionTarget::IGNORE_AFY |
                                // mavros_msgs::PositionTarget::IGNORE_AFZ |
                                // mavros_msgs::PositionTarget::IGNORE_YAW |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE; // You command each parameter that is not ignored. i.e. that is commented out

    // setpoint_raw.header.frame_id = "map";
    setpoint_raw.header.stamp = ros::Time::now();

    // Frequency of Node
    float node_freq = 30.0;
    ros::Rate rate(node_freq);
    float dt = 1/node_freq;

    // Get params ??? Add code to get this from FCU
    float param_mpc_xy_p = 0.8;
    float param_mpc_xy_vel_p_acc = 0.09;
    float param_mpc_xy_vel_i_acc = 0.0; //0.02;
    float param_mpc_xy_vel_d_acc = 0.0; //0.01;        

    float param_mpc_z_p  = 1.0;
    float param_mpc_z_vel_p_acc = 0.2;
    float param_mpc_z_vel_i_acc = 0.0; //0.02;
    float param_mpc_z_vel_d_acc = 0.0;

    // Initialise integral count
    double vel_err_int_x = 0.0;
    double vel_err_int_y = 0.0;
    double vel_err_int_z = 0.0;
    
    // Derivative of velocity
    double vel_derv_x = 0.0;
    double vel_derv_y = 0.0;
    double vel_derv_z = 0.0;

    // Declare variables:
    double pos_sp_x; // Local_NED position setpoint
    double pos_sp_y;
    double pos_sp_z;

    double pos_x; // Local_NED position feedback
    double pos_y;
    double pos_z;

    double vel_sp_x;
    double vel_sp_y;
    double vel_sp_z;

    double vel_x = local_velocity.twist.linear.x; // Local_NED velocity feedback
    double vel_y = local_velocity.twist.linear.y;
    double vel_z = local_velocity.twist.linear.z;

    double vel_err_x;
    double vel_err_y;
    double vel_err_z;

    double acc_sp_x;
    double acc_sp_y;
    double acc_sp_z;  

    double target_vel_sp_x;
    double target_vel_sp_y;
    double target_vel_sp_z;

    double diff_vel_x;
    double diff_vel_y;
    double diff_vel_z;

    double target_acc_sp_x;
    double target_acc_sp_y;
    double target_acc_sp_z;

    double diff_acc_x;
    double diff_acc_y;
    double diff_acc_z;
    
    ROS_INFO("Started offboard position control");
    while(ros::ok()){        

        // Update derivative
        vel_derv_x = (local_velocity.twist.linear.x - vel_x) / dt; // (Current vel - previous vel) / change in time
        vel_derv_y = (local_velocity.twist.linear.y - vel_y) / dt;
        vel_derv_z = (local_velocity.twist.linear.z - vel_z) / dt;

        // Set readable names:
        pos_sp_x = received_setpoint.position.x;
        pos_sp_y = received_setpoint.position.y;
        pos_sp_z = received_setpoint.position.z;

        pos_x = local_position.pose.position.x;
        pos_y = local_position.pose.position.y;
        pos_z = local_position.pose.position.z;

        vel_x = local_velocity.twist.linear.x;
        vel_y = local_velocity.twist.linear.y;
        vel_z = local_velocity.twist.linear.z;
        
        // Coordinate frame notes: 
        // - Input to waypoints_sheduler.py is in NED frame
        // - waypoints_sheduler.py publishes in ENU frame
        // - pos_control_node.cpp receives and publishes in ENU frame

        // Hardcode input
        // received_setpoint.position.x = 0;
        // received_setpoint.position.y = 0;
        // received_setpoint.position.z = 5;
        // received_setpoint.yaw = deg2rad(90);

        // Position controller:
        // --------------------

        // setpoint_raw.velocity.x = (received_setpoint.position.x - local_position.pose.position.x)*param_mpc_xy_p;
        // setpoint_raw.velocity.y = (received_setpoint.position.y - local_position.pose.position.y)*param_mpc_xy_p;
        // setpoint_raw.velocity.z = (received_setpoint.position.z - local_position.pose.position.z)*param_mpc_z_p;  
        
        vel_sp_x = (pos_sp_x - local_position.pose.position.x)*param_mpc_xy_p;
        vel_sp_y = (pos_sp_y - local_position.pose.position.y)*param_mpc_xy_p;
        vel_sp_z = (pos_sp_z - local_position.pose.position.z)*param_mpc_z_p;  

        // ROS_INFO("pos_sp_z:%.2f, pos_z:%.2f", received_setpoint.position.z, local_position.pose.position.z );
        
        // Velocity controller:
        // -------------------

        // Calculate error
        vel_err_x = vel_sp_x - vel_x;
        vel_err_y = vel_sp_y - vel_y;
        vel_err_z = vel_sp_z - vel_z;
        // ROS_INFO("err: x:%f, y:%f, z:%f", vel_err_x, vel_err_y, vel_err_z );
        
        // PID control
        // acc_sp_x =    param_mpc_xy_vel_p * vel_err_x 
        //             + param_mpc_xy_vel_i * vel_err_int_x
        //             + param_mpc_xy_vel_d * vel_derv_x;

        // acc_sp_y =    param_mpc_xy_vel_p * vel_err_y 
        //             + param_mpc_xy_vel_i * vel_err_int_y
        //             + param_mpc_xy_vel_d * vel_derv_y;

        // acc_sp_z =    param_mpc_z_vel_p * vel_err_z 
        //             + param_mpc_z_vel_i * vel_err_int_z
        //             + param_mpc_z_vel_d * vel_derv_z;

        // Update integral
        vel_err_int_x += vel_err_x * dt;
        vel_err_int_y += vel_err_y * dt;
        vel_err_int_z += vel_err_z * dt;

        // Populate setpoint (ignore-tags determine which are used):
        // setpoint_raw.position.x = pos_sp_x;
        // setpoint_raw.position.y = pos_sp_y;
        // setpoint_raw.position.z = pos_sp_z;
        
        // setpoint_raw.velocity.x = vel_sp_x;
        // setpoint_raw.velocity.y = vel_sp_y;
        // setpoint_raw.velocity.z = vel_sp_z;

        // setpoint_raw.acceleration_or_force.x = acc_sp_x;
        // setpoint_raw.acceleration_or_force.y = acc_sp_y;
        // setpoint_raw.acceleration_or_force.z = acc_sp_z;
        setpoint_raw.acceleration_or_force.x = 10; // East local (not body)
        setpoint_raw.acceleration_or_force.y = 0; // North local (not body)
        setpoint_raw.acceleration_or_force.z = 0; // Up local (not body)

        setpoint_raw.yaw = received_setpoint.yaw; // 0 = East local. Direct feed-through from waypoints_sheduler.py node

        // Test difference in setpoints
        target_vel_sp_x = target_setpoint_raw.velocity.x;
        target_vel_sp_y = target_setpoint_raw.velocity.y;
        target_vel_sp_z = target_setpoint_raw.velocity.z;

        diff_vel_x = target_vel_sp_x - vel_sp_x;
        diff_vel_y = target_vel_sp_y - vel_sp_y;
        diff_vel_z = target_vel_sp_z - vel_sp_z;

        target_acc_sp_x = target_setpoint_raw.acceleration_or_force.x;
        target_acc_sp_y = target_setpoint_raw.acceleration_or_force.y;
        target_acc_sp_z = target_setpoint_raw.acceleration_or_force.z;

        diff_acc_x = target_acc_sp_x - acc_sp_x;
        diff_acc_y = target_acc_sp_y - acc_sp_y;
        diff_acc_z = target_acc_sp_z - acc_sp_z;

        // ROS_INFO("x: %.5f, y: %.5f, z: %.5f", diff_vel_x, diff_vel_y, diff_vel_z);
        // ROS_INFO("x: %.3f, y: %.3f, z: %.3f", diff_acc_x, diff_acc_y, diff_acc_z);
        // ROS_INFO("x: %.3f, y: %.3f, z: %.3f", target_acc_sp_x, target_acc_sp_y, target_acc_sp_z);

        // Publish setpoint:    
        setpoint_raw_pub.publish(setpoint_raw); // Publish to MAVROS
        local_setpoint_raw_pub.publish(setpoint_raw); // Publish to local ROS topic, not MAVROS

        // ??? compare offboard setpoint to onboard mavros/setpoint_raw/target_local

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}