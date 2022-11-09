#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

char getch()
{
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return (buf);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "forklift_keyboard_control_node");
    ros::NodeHandle node_handler;
    std::string robot_model_name = "/EVX_14";

    ros::Publisher left_joint_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/wheel_left_joint_controller/command", 1000);
    ros::Publisher right_joint_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/wheel_right_joint_controller/command", 1000);
    ros::Publisher left_steer_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/wheel_left_steering_joint_controller/command", 1000);
    ros::Publisher right_steer_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/wheel_right_steering_joint_controller/command", 1000);
    ros::Publisher fork_load_joint_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/pris_joint_controller/command", 1000);

    ros::Rate loop_rate(10);
    bool terminate = false;
    float drive_speed = 0.0;
    float steer_angle = 0.0;
    float forkload_height = 0.0;
    float drive_step = 0.2;
    float steer_step = 0.1;
    float forkload_step = 0.2;
    float steer_limit = 1.0;

    std_msgs::Float64 left_joint_msgs;
    std_msgs::Float64 right_joint_msgs;
    std_msgs::Float64 left_steer_msgs;
    std_msgs::Float64 right_steer_msgs;
    std_msgs::Float64 forkload_height_msgs;

    char input;

    // Echo input:
    std::cout << "press the following keys to control forklift" << std::endl;
    std::cout << "   w    --for forward" << std::endl;
    std::cout << "a     s  --for left and right" << std::endl;
    std::cout << "   x    --for backward" << std::endl;
    std::cout << "s --for stop" << std::endl;
    std::cout << "c --for killing the node" << std::endl;
    while (ros::ok() && terminate == false)
    {
        input = getch();
        switch (input)
        {
        case 'w':
            drive_speed += drive_step;
            break;
        case 'x':
            drive_speed -= drive_step;
            break;
        case 'a':
            steer_angle += steer_step;
            if (steer_angle > steer_limit)
            {
                steer_angle = steer_limit;
            }
            break;
        case 'd':
            steer_angle -= steer_step;
            if (steer_angle < -steer_limit)
            {
                steer_angle = -steer_limit;
            }
            break;
        case 's':
            drive_speed = 0;
            // steer_angle = 0;
            break;
        case 'q':
            forkload_height += forkload_step;
            break;
        case 'z':
            forkload_height -= forkload_step;
            break;
        case 'c':
            terminate = true;
            break;
        default:
            break;
        }
        left_joint_msgs.data = -drive_speed;
        right_joint_msgs.data = -drive_speed;
        left_steer_msgs.data = -steer_angle;
        right_steer_msgs.data = -steer_angle;
        forkload_height_msgs.data = forkload_height;

        std_msgs::String msg;
        std::stringstream ss;
        msg.data = ss.str();
        std::cout << "moving with speed " << drive_speed;
        std::cout << "   steering angle " << steer_angle << std::endl;
        std::cout << "   ------------- " << std::endl;
        left_joint_pub.publish(left_joint_msgs);
        right_joint_pub.publish(right_joint_msgs);
        left_steer_pub.publish(left_steer_msgs);
        right_steer_pub.publish(right_steer_msgs);
        fork_load_joint_pub.publish(forkload_height_msgs);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
