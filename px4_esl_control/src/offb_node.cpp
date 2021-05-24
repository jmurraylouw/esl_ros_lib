/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

// Subscriber callback functions and variables
mavros_msgs::State current_state;
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
mavros_msgs::PositionTarget received_setpoint; // Received setpoint from mpc_waypoints_node
void setpoint_callback(const mavros_msgs::PositionTarget::ConstPtr& msg){
    received_setpoint = *msg;
    ROS_INFO("Position setpoint: [x: %f, y: %f, z: %f]", (*msg).position.x, (*msg).position.y, (*msg).position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_callback);
    ros::Subscriber received_setpoint_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("/setpoint_raw/local", 10, setpoint_callback); // Received setpoint from mpc_waypoints

    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher vel_sp_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Publisher accel_sp_pub = nh.advertise<geometry_msgs::Vector3Stamped>
            ("mavros/setpoint_accel/accel", 10); // Message buffer of 10, i.e. 10 messages are kept before throwing away

    // ??? Maybe need to publish yaw setpoint also

    // Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // Frequency of Node
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pos_sp; // Position set_point
    pos_sp.pose.position.x = 0;
    pos_sp.pose.position.y = 0;
    pos_sp.pose.position.z = 20;

    geometry_msgs::Twist vel_sp; // Velocity setpoint
    // MAVROS message:  setpoint_velocity/cmd_vel_unstamped (geometry_msgs/Twist)
    vel_sp.linear.x = 0;
    vel_sp.linear.y = 0.5;
    vel_sp.linear.z = 0.5;

    geometry_msgs::Vector3Stamped accel_sp; // Acceleration setpoint as a ROS message: http://docs.ros.org/en/api/geometAcceleration setpoint as a ROS messagery_msgs/html/msg/Vector3Stamped.html
    accel_sp.vector.x = 0;
    accel_sp.vector.y = 0;
    accel_sp.vector.z = 100;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        // local_pos_pub.publish(pose);
        vel_sp_pub.publish(vel_sp);
        // accel_sp_pub.publish(accel_sp);

        ros::spinOnce();
        rate.sleep();
    }

    // Set offboard mode and arm
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // ??? Figure out which node starts arming and offboard mode etc.


        // Subscribe to the waypoints ROS topic
        // pos_sp = // position setpoint

        // Subscribe to pendulum angle topic

        // Subscribe to state of drone topic
        // pos_sp_vect = Vector3f(pos_sp.pose.position.x, pos_sp.pose.position.y, pos_sp.pose.position.z);
        // pos_vect = Vector3f();

        // // Get params
        // mpc_xy_p = 0.95;
        // mpc_z_p  = 1.0;

        // // P-position controller
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


        // Publish setpoint to MAVROS
        // local_pos_pub.publish(pose);
        vel_sp_pub.publish(vel_sp);
        // accel_sp_pub.publish(accel_sp);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
