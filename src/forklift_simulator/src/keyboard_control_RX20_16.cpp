#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
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

    ros::init(argc, argv, "RX20_16_keyboard_control_node");
    ros::NodeHandle node_handler;
    std::string robot_model_name = "/RX20_16";

    // ros::Publisher left_joint_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/wheel_left_driving_joint_controller/command", 1000);
    // ros::Publisher right_joint_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/wheel_right_driving_joint_controller/command", 1000);
    // ros::Publisher left_steer_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/wheel_left_steering_joint_controller/command", 1000);
    // ros::Publisher right_steer_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/wheel_right_steering_joint_controller/command", 1000);
    ros::Publisher cmd_vel_pub = node_handler.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Publisher fork_tilt_joint_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/tilt_joint_controller/command", 1000);
    ros::Publisher fork_lift_joint_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/lift_joint_controller/command", 1000);
    ros::Publisher fork_sideshift_joint_pub = node_handler.advertise<std_msgs::Float64>(robot_model_name + "/sideshift_joint_controller/command", 1000);

    ros::Rate loop_rate(10);
    bool terminate = false;
    float drive_speed = 0.0;
    float steer_angle = 0.0;
    float forklift_tilt = 0.0;
    float forklift_lift = 0.0;
    float forklift_sideshift = 0.0;
    float drive_step = 0.2;
    float steer_step = 0.1;
    float forklift_tilt_step = 0.02;
    float forklift_lift_step = 0.05;
    float forklift_sideshift_step = 0.05;
    float steer_limit = 1.0;

    // std_msgs::Float64 left_joint_msgs;
    // std_msgs::Float64 right_joint_msgs;
    // std_msgs::Float64 left_steer_msgs;
    // std_msgs::Float64 right_steer_msgs;
    geometry_msgs::Twist cmd_vel_msgs;
    std_msgs::Float64 forklift_tilt_msgs;
    std_msgs::Float64 forklift_lift_msgs;
    std_msgs::Float64 forklift_sideshift_msgs;

    char input;

    // Echo input:
    std::cout << "press the following keys to control forklift" << std::endl;
    std::cout << "   w    --for forward" << std::endl;
    std::cout << "a     s  --for left and right" << std::endl;
    std::cout << "   x    --for backward" << std::endl;
    std::cout << "q and z  --for lift up down" << std::endl;
    std::cout << "e and r --for sideshift left right" << std::endl;
    std::cout << "f and v --for tilt forward and backward" << std::endl;
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
            forklift_lift += forklift_lift_step;
            break;
        case 'z':
            forklift_lift -= forklift_lift_step;
            break;
        case 'e':
            forklift_sideshift += forklift_sideshift_step;
            break;
        case 'r':
            forklift_sideshift -= forklift_sideshift_step;
            break;
        case 'f':
            forklift_tilt += forklift_tilt_step;
            break;
        case 'v':
            forklift_tilt -= forklift_tilt_step;
            break;
        case 'c':
            terminate = true;
            break;
        default:
            break;
        }
        // left_joint_msgs.data = -drive_speed;
        // right_joint_msgs.data = -drive_speed;
        // left_steer_msgs.data = -steer_angle;
        // right_steer_msgs.data = -steer_angle;
        cmd_vel_msgs.linear.x = drive_speed;
        cmd_vel_msgs.angular.z = steer_angle;
        forklift_lift_msgs.data = forklift_lift;
        forklift_tilt_msgs.data = forklift_tilt;
        forklift_sideshift_msgs.data = forklift_sideshift;

        std_msgs::String msg;
        std::stringstream ss;
        msg.data = ss.str();
        std::cout << "moving with speed " << drive_speed;
        std::cout << "   steering angle " << steer_angle << std::endl;
        std::cout << "   ------------- " << std::endl;
        // left_joint_pub.publish(left_joint_msgs);
        // right_joint_pub.publish(right_joint_msgs);
        // left_steer_pub.publish(left_steer_msgs);
        // right_steer_pub.publish(right_steer_msgs);
        cmd_vel_pub.publish(cmd_vel_msgs);
        fork_lift_joint_pub.publish(forklift_lift_msgs);
        fork_tilt_joint_pub.publish(forklift_tilt_msgs);
        fork_sideshift_joint_pub.publish(forklift_sideshift_msgs);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
