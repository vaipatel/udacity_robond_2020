#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

enum DIRECTION { STOP, LEFT, FORWARD, RIGHT };

// Define global client
ros::ServiceClient client;

// Helper to make client call command_robot srvc
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Command robot to move");

    // Make the request object
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// Identify ball, send requests to drive_bot to move bot to ball
void process_image_cbk(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int left = img.step / 3;
    int right = 2 * left;
    DIRECTION direction = DIRECTION::STOP;
    
    // I'm assuming data is laid out row by row. How to confirm?
    for (int i = 0; i < img.height * img.step; ++i)
    {
        if (img.data[i] == white_pixel)
        {
            int j = i % img.step;
            if (j < left)
                direction = DIRECTION::LEFT;
            else if (j < right)
                direction = DIRECTION::FORWARD;
            else
                direction = DIRECTION::RIGHT;
            
            break;
        }
    }
    float lin_x = 0.0, lin_x_max = 0.2;
    float ang_z = 0.0, ang_z_max = 0.1;

    // For any direction other than STOP, we move forward
    if (direction != DIRECTION::STOP)
        lin_x = lin_x_max;

    // If we have to turn LEFT, make ang_z positive. Oppo for right.
    if (direction == DIRECTION::LEFT)
        ang_z = ang_z_max;
    else if (direction == DIRECTION::RIGHT)
        ang_z = -ang_z_max;

    // Drive with lin_x and ang_z velocities.
    drive_robot(lin_x, ang_z);
}

int main(int argc, char *argv[])
{
    // Init node
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client for sending requests to command_robot service
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    
    // Sub to get image data with queue size 10, process images and react with process_image_cbk
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 10, process_image_cbk);

    // Handle ROS comm events
    ros::spin();

    return 0;
}