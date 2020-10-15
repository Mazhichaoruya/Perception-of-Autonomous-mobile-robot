#ifndef _SERIAL_NODE_H_
#define _SERIAL_NODE_H_

#include "ros/ros.h"
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>



class SerialCommunication
{
public:
    SerialCommunication() {}
    SerialCommunication(ros::NodeHandle &nh);

private:
    const double pi = 3.1415926536;
    #define RadToDegree(rad) (rad / pi) * 180
    #define DegreeToRad(degree) (degree / 180) * pi

    ros::Publisher odom_pub_;
    ros::Subscriber cmd_sub_;
    ros::Timer serial_timer_;
    serial::Serial ros_ser_;

    union Rcv_Data 
    {
        unsigned char rcv_buf[17];
        struct
        {
            unsigned char header_first;
            unsigned char header_second;
            unsigned char control_mode;
            unsigned char status;
            float pos_x;
            float pos_y;
            float yaw;
            unsigned char check;
        } sensor_data;
    } rcv_data_;

    union Send_Data 
    {
        unsigned char send_buf[17];
        struct
        {
            unsigned char header_first;
            unsigned char header_second;
            unsigned char command;
            unsigned char status;
            float vel_x;
            float vel_y;
            float vel_th;
            unsigned char check;
        } vel_command;
    } send_data_;

    tf::TransformBroadcaster odom_broadcaster_;
    geometry_msgs::TransformStamped odom_trans_;  
    nav_msgs::Odometry odom_;
    ros::Time current_time_, last_time_;

    double pos_x_, last_pos_x_;
    double pos_y_, last_pos_y_;
    double yaw_, last_yaw_;

    void cmdVelCallback(const geometry_msgs::Twist &cmd_vel);
    void serialTimerCallback(const ros::TimerEvent &e);
};

#endif