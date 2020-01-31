#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS publisher of motor commands
ros::Publisher motor_command_publisher;


bool handle_drive_request(ball_chaser::DriveToTarget::Request &req,
    ball_chaser::DriveToTarget::Response &res)
{
    float linear_x = (float)req.linear_x;
    float angular_z = (float)req.angular_z;
    ROS_INFO("DriveToTarget::Request received - lx:%1.2f, az:%1.2f", linear_x, angular_z);

    // Create a motor command of type Twist
    geometry_msgs::Twist motor_command;

    // Set velocities
    motor_command.linear.x = linear_x;
    motor_command.angular.z = angular_z;

    motor_command_publisher.publish(motor_command);

    // I don't know if we need to wait here.
    // ros::Duration(1).sleep();

    res.msg_feedback = "Wheel velocities set - lx: " + std::to_string(linear_x) + " az: " + std::to_string(angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char *argv[])
{
    // Init node
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Define motor command publisher of queue_size 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define service to handle drive requests
    ros::ServiceServer srvc = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ROS_INFO("Ready to receive and forward drive requests and send back velocities");

    // Handle ROS comm events
    ros::spin();

    return 0;
}