/**
 * @file offb_node.cpp
 * @brief MPC node for a quarotor running PX4.
 * Input: Local position setpoint (ROS topic: /setpoint_raw/local (not in mavros))
 * Output: Acceleration setpoint (ROS topic: mavros/setpoint_raw/local)
 * First run mpc_waypoints.py to publish and update position setpoint to local ROS topic
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

// Subscriber callback functions and variables
mavros_msgs::State current_state;
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
mavros_msgs::PositionTarget received_setpoint; // Received setpoint from mpc_waypoints_node
void setpoint_callback(const mavros_msgs::PositionTarget::ConstPtr& msg){
    received_setpoint = *msg;
    // ROS_INFO("Position setpoint: [x: %f, y: %f, z: %f]", (*msg).position.x, (*msg).position.y, (*msg).position.z);
}
geometry_msgs::PoseStamped local_position;
void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
    // ROS_INFO("Local position: [x: %f, y: %f, z: %f]", (*msg).pose.position.x, (*msg).pose.position.y, (*msg).pose.position.z);
}
double heading;
double heading_local; // Heading relative to initial heading
double heading_init = 0; // Initial heading
void heading_callback(const std_msgs::Float64::ConstPtr& msg){
    heading = (*msg).data;
    heading_local = heading - heading_init;
    // ROS_INFO("Heading: %f", (*msg).data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher vel_sp_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Publisher accel_sp_pub = nh.advertise<geometry_msgs::Vector3Stamped>
            ("mavros/setpoint_accel/accel", 10);
    ros::Publisher setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_callback);
    ros::Subscriber received_setpoint_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("/setpoint_raw/local", 10, setpoint_callback); // Received setpoint from mpc_waypoints
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_callback);
    ros::Subscriber heading_sub = nh.subscribe<std_msgs::Float64>
            ("mavros/global_position/compass_hdg", 10, heading_callback);

    // ??? Maybe need to publish yaw setpoint also

    // Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    double heading_init = heading; // Save initial heading

    // Initialise publisher messages:

    // geometry_msgs::PoseStamped pos_sp; // Position set_point
    // pos_sp.pose.position.x = 0;
    // pos_sp.pose.position.y = 0;
    // pos_sp.pose.position.z = 20;

    // geometry_msgs::Twist vel_sp; // Velocity setpoint
    // vel_sp.linear.x = 0;
    // vel_sp.linear.y = 0.5;
    // vel_sp.linear.z = 0;

    // geometry_msgs::Vector3Stamped accel_sp; // Acceleration setpoint as a ROS message: http://docs.ros.org/en/api/geometAcceleration setpoint as a ROS messagery_msgs/html/msg/Vector3Stamped.html
    // accel_sp.vector.x = 0.1;
    // accel_sp.vector.y = 0;
    // accel_sp.vector.z = 0;

    mavros_msgs::PositionTarget setpoint_raw;
    setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    setpoint_raw.type_mask =    mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                // mavros_msgs::PositionTarget::IGNORE_VX |
                                // mavros_msgs::PositionTarget::IGNORE_VY |
                                // mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

// uint16 IGNORE_PX = 1 # Position ignore flags
// uint16 IGNORE_PY = 2
// uint16 IGNORE_PZ = 4
// uint16 IGNORE_VX = 8 # Velocity vector ignore flags
// uint16 IGNORE_VY = 16
// uint16 IGNORE_VZ = 32
// uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
// uint16 IGNORE_AFY = 128
// uint16 IGNORE_AFZ = 256
// uint16 FORCE = 512 # Force in af vector flag
// uint16 IGNORE_YAW = 1024
// uint16 IGNORE_YAW_RATE = 2048

    setpoint_raw.header.frame_id = "world";
    setpoint_raw.header.stamp = ros::Time::now();
    
    // setpoint_raw.position.x = 1;
    // setpoint_raw.position.y = 0;
    // setpoint_raw.position.z = 4;

    ROS_INFO("frame: %i", setpoint_raw.coordinate_frame);
    ROS_INFO("check ignore_pz: %i", setpoint_raw.type_mask & mavros_msgs::PositionTarget::IGNORE_PZ);
    ROS_INFO("check ignore_vx: %i", setpoint_raw.type_mask & mavros_msgs::PositionTarget::IGNORE_VX);
    ROS_INFO("type_mask: %i", setpoint_raw.type_mask);

    setpoint_raw.velocity.x = 0.0;
    setpoint_raw.velocity.y = 1.0;
    setpoint_raw.velocity.z = 1.0;

    // setpoint_raw.acceleration_or_force.x = nan;
    // setpoint_raw.acceleration_or_force.y = nan;
    // setpoint_raw.acceleration_or_force.z = nan;

    // Frequency of Node
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        // local_pos_pub.publish(pose);
        // vel_sp_pub.publish(vel_sp);
        // accel_sp_pub.publish(accel_sp);
        setpoint_raw_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    // Set offboard mode and arm
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    if( current_state.mode != "OFFBOARD" ){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
    }

    while(ros::ok()){

        // Inputs:
        // received_setpoint
        // ??? pendulum angle
        // local_position
        // velocity

        // Subscribe to pendulum angle topic
        // Subscribe to state of drone topic

        // Get params
        double mpc_xy_p = 0.95;
        double mpc_z_p  = 1.0;

        // Position controller
        // received_setpoint.position.x = 0;
        // received_setpoint.position.y = 0;
        // received_setpoint.position.z = 5;
        // Local velocity setpoint:
        // double vel_sp_x_local = (received_setpoint.position.x - local_position.pose.position.x)*mpc_xy_p;
        // double vel_sp_y_local = (received_setpoint.position.y - local_position.pose.position.z)*mpc_xy_p;
        // double vel_sp_z_local = (received_setpoint.position.z - local_position.pose.position.z)*mpc_z_p;        
        // double vel_sp_x_local = 1.0;
        // double vel_sp_y_local = 0.0;
        // double vel_sp_z_local = 0.0;
        // // Transform to global velocity and publish:
        // vel_sp.linear.x = vel_sp_x_local*cos(heading * M_PI/180) - vel_sp_y_local*sin(heading * M_PI/180);
        // vel_sp.linear.y = vel_sp_x_local*sin(heading * M_PI/180) + vel_sp_y_local*cos(heading * M_PI/180);
        // vel_sp.linear.z = vel_sp_z_local;
        // vel_sp_pub.publish(vel_sp);

        // P-position controller
        // const Vector3f vel_sp_position = (pos_sp_vect - pos_vect).emult(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(),
        //                 _param_mpc_z_p.get()));
        // vel_sp_vect = vel_sp_position + vel_sp;
        // // Constrain horizontal velocity by prioritizing the velocity component along the
        // // the desired position setpoint over the feed-forward term.
        // const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
        //             Vector2f(_vel_sp - vel_sp_position), _param_mpc_xy_vel_max.get());
        // vel_sp(0) = vel_sp_xy(0);
        // vel_sp(1) = vel_sp_xy(1);
        // // Constrain velocity in z-direction.
        // vel_sp(2) = math::constrain(vel_sp(2), -_constraints.speed_up, _constraints.speed_down);

        // Apply MPC velocity control
        // accel_sp = // velocity setpoint

        // Publish setpoint to MAVROS:
        setpoint_raw_pub.publish(setpoint_raw);

        // local_pos_pub.publish(pose);
        // vel_sp_pub.publish(vel_sp);
        // accel_sp_pub.publish(accel_sp);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
