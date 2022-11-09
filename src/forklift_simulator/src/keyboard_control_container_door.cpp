#include "ros/ros.h"
#include "std_msgs/Float64.h"
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

    ros::init(argc, argv, "container_keyboard_control_node");
    ros::NodeHandle node_handler;

    ros::Publisher left_door_pub = node_handler.advertise<std_msgs::Float64>("/red_container/left_door_joint_controller/command", 1000);
    ros::Publisher right_door_pub = node_handler.advertise<std_msgs::Float64>("/red_container/right_door_joint_controller/command", 1000);

    ros::Rate loop_rate(10);
    bool terminate = false;
    float left_angle = 0.0;
    float right_angle = 0.0;
    float angle_step = 0.1;

    float angle_limit = 1.57;

    std_msgs::Float64 left_door_msgs;
    std_msgs::Float64 right_door_msgs;

    char input;

    // Echo input:
    std::cout << "press the following keys to control container door" << std::endl;
    std::cout << "  q  w    --for left door open and close" << std::endl;
    std::cout << "  e  r    --for right door open and close" << std::endl;

    std::cout << "c --for killing the node" << std::endl;
    while (ros::ok() && terminate == false)
    {
        input = getch();
        switch (input)
        {

        case 'w':
            left_angle += angle_step;
            if (left_angle > angle_limit)
            {
                left_angle = angle_limit;
            }
            break;
        case 'q':
            left_angle -= angle_step;
            if (left_angle < -angle_limit)
            {
                left_angle = -angle_limit;
            }
            break;

        case 'r':
            right_angle += angle_step;
            if (right_angle > angle_limit)
            {
                right_angle = angle_limit;
            }
            break;
        case 'e':
            right_angle -= angle_step;
            if (right_angle < -angle_limit)
            {
                right_angle = -angle_limit;
            }
            break;
        case 'c':
            terminate = true;
            break;
        default:
            break;
        }
        left_door_msgs.data = left_angle;
        right_door_msgs.data = right_angle;

        std::cout << "left and right angle  " << left_angle << " " << right_angle << std::endl;
        std::cout << "   ------------- " << std::endl;
        left_door_pub.publish(left_door_msgs);
        right_door_pub.publish(right_door_msgs);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
