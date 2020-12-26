#include "serial_node.h"

using namespace std;

SerialCommunication::SerialCommunication(ros::NodeHandle &handle)
{
    cmd_sub_ = handle.subscribe("cmd_vel", 1, &SerialCommunication::cmdVelCallback, this);
    odom_pub_ = handle.advertise<nav_msgs::Odometry>("odom", 1);

    serial_timer_ = handle.createTimer(ros::Duration(0.01), &SerialCommunication::serialTimerCallback, this); // 10ms读取串口数据

    try // 检测串口状态
    {
        ros_ser_.setPort("/dev/ttyUSB0");
        ros_ser_.setBaudrate(115200);
        serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
        ros_ser_.setTimeout(time_out);
        ros_ser_.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("[SerialNode]: Unable to open port ");
    }
    if (ros_ser_.isOpen())
    {
        ROS_INFO_STREAM("[SerialNode]: Serial Port opened");
    }

    odom_trans_.header.frame_id = "odom";
    odom_trans_.child_frame_id = "base_link";

    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_link";

    current_time_ = last_time_ = ros::Time::now();
    pos_x_ = 0.0;
    pos_y_ = 0.0;
    yaw_ = 0.0;
}

void SerialCommunication::cmdVelCallback(const geometry_msgs::Twist &cmd_vel)
{
    send_data_.vel_command.header_first = 0x07;
    send_data_.vel_command.header_second = 0x31;
    send_data_.vel_command.command = 0x01;
    send_data_.vel_command.status = 0x02;
    send_data_.vel_command.vel_x = cmd_vel.linear.x;
    send_data_.vel_command.vel_y = cmd_vel.linear.y;
    send_data_.vel_command.vel_th = cmd_vel.angular.z;
    send_data_.vel_command.check = send_data_.send_buf[2]^send_data_.send_buf[3]^send_data_.send_buf[4]
                                    ^send_data_.send_buf[5]^send_data_.send_buf[6]^send_data_.send_buf[7]
                                    ^send_data_.send_buf[8]^send_data_.send_buf[9]^send_data_.send_buf[10]
                                    ^send_data_.send_buf[11]^send_data_.send_buf[12]^send_data_.send_buf[13]
                                    ^send_data_.send_buf[14]^send_data_.send_buf[15];
    ros_ser_.write(send_data_.send_buf, sizeof(send_data_.send_buf));
    cout<<"KEY:"<<cmd_vel.linear.x<<","<<cmd_vel.linear.y<<","<<cmd_vel.angular.z<<",rate="<<cmd_vel.linear.z<<endl;
    // cout << "[SerialNode]: Write to serial port successfully" << endl;
//    ROS_INFO_STREAM("[SerialNode]: Write to serial port successfully");
}

void SerialCommunication::serialTimerCallback(const ros::TimerEvent &e) // 10ms定时器
{
    if (ros_ser_.available())
    {
        // ROS_INFO_STREAM("Reading from serial port");
        // ros_ser.flushInput();   //清除串口FIFO的缓存
        ros_ser_.read(rcv_data_.rcv_buf, sizeof(rcv_data_.rcv_buf));
        if (rcv_data_.sensor_data.header_first == 0x07 && rcv_data_.sensor_data.header_second == 0x31)
        {
            unsigned char check_result = rcv_data_.rcv_buf[2]^rcv_data_.rcv_buf[3]^rcv_data_.rcv_buf[4] 
                                            ^rcv_data_.rcv_buf[5]^rcv_data_.rcv_buf[6]^rcv_data_.rcv_buf[7]    
                                            ^rcv_data_.rcv_buf[8]^rcv_data_.rcv_buf[9]^rcv_data_.rcv_buf[10]   
                                            ^rcv_data_.rcv_buf[11]^rcv_data_.rcv_buf[12]^rcv_data_.rcv_buf[13] 
                                            ^rcv_data_.rcv_buf[14]^rcv_data_.rcv_buf[15];
            if (check_result == rcv_data_.sensor_data.check)
            {

                ROS_INFO_STREAM("Reading from stm32 : " << rcv_data_.sensor_data.pos_x << ","
                                                        << rcv_data_.sensor_data.pos_y << "," 
                                                        << rcv_data_.sensor_data.yaw);

                last_pos_x_ = pos_x_; //记录上一次位姿
                last_pos_y_ = pos_y_;
                last_yaw_ = yaw_;

                pos_x_ = rcv_data_.sensor_data.pos_x;
                pos_y_ = rcv_data_.sensor_data.pos_y;
                yaw_ = DegreeToRad(rcv_data_.sensor_data.yaw);

                last_time_ = current_time_;
                current_time_ = ros::Time::now();
                double dt = (current_time_ - last_time_).toSec();

                double vel_x = (pos_x_ - last_pos_x_) / dt;
                double vel_y = (pos_y_ - last_pos_y_) / dt;
                double vel_th = (yaw_ - last_yaw_) / dt;

                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_);

                odom_trans_.header.stamp = current_time_;
                odom_trans_.transform.translation.x = rcv_data_.sensor_data.pos_x;
                odom_trans_.transform.translation.y = rcv_data_.sensor_data.pos_y;
                odom_trans_.transform.translation.z = 0.0;
                odom_trans_.transform.rotation = odom_quat;
                odom_broadcaster_.sendTransform(odom_trans_); //发布tf变换

                odom_.header.stamp = current_time_;
                odom_.pose.pose.position.x = rcv_data_.sensor_data.pos_x;
                odom_.pose.pose.position.y = rcv_data_.sensor_data.pos_y;
                odom_.pose.pose.position.z = 0.0;
                odom_.pose.pose.orientation = odom_quat;

                odom_.twist.twist.linear.x = vel_x;
                odom_.twist.twist.linear.y = vel_y;
                odom_.twist.twist.angular.z = vel_th;
                odom_pub_.publish(odom_); //发布odom
            }
            else
            {
                ROS_INFO_STREAM("[SerialNode]: Data transfer error!");
            }
        }
        else
        {
            ROS_INFO_STREAM("[SerialNode]: Data is not aligned!");
        }
    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_communication_node");
    ros::NodeHandle nh;

    SerialCommunication serial_communication(nh);

    ros::spin();
    return 0;
}
