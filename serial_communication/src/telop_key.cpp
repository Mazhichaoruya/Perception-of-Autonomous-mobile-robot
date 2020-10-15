//
// Created by mzc on 2020/10/5.
//
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

class SmartCarKeyboardTeleopNode
{
private:
    double walk_vel_;
    double run_vel_;
    double yaw_rate_;
    double yaw_rate_run_;

    geometry_msgs::Twist cmdvel_;
    ros::NodeHandle n_;
    ros::Publisher pub_;

public:
    SmartCarKeyboardTeleopNode()
    {
        pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        ros::NodeHandle n_private("~");
        n_private.param("walk_vel", walk_vel_, 0.1);
        n_private.param("run_vel", run_vel_, 0.01);
        n_private.param("yaw_rate", yaw_rate_, 0.1);
        n_private.param("yaw_rate_run", yaw_rate_run_, 0.01);
    }

    ~SmartCarKeyboardTeleopNode() { }
    void keyboardLoop();

    void stopRobot()
    {
        cmdvel_.linear.x = 0.0;
        cmdvel_.linear.y=0.0;
        cmdvel_.angular.z = 0.0;
        pub_.publish(cmdvel_);
    }
};

SmartCarKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

void SmartCarKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double max_tv1 = walk_vel_;
    double max_tv2 = walk_vel_;
    double max_rv = yaw_rate_;
    bool dirty = false;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        int speed1 = 0,speed2=0;
        int turn = 0;
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
            }

            continue;
        }

        switch(c)
        {
            case KEYCODE_W:
                max_tv1 = walk_vel_;
                speed1 = 1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_S:
                max_tv1 = walk_vel_;
                speed1 = -1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_A:
                max_rv = yaw_rate_;
                turn = 1;
                dirty = true;
                break;
            case KEYCODE_D:
                max_rv = yaw_rate_;
                turn = -1;
                dirty = true;
                break;

            case KEYCODE_W_CAP:
                if (walk_vel_<0.5)
                    walk_vel_+=run_vel_;
                if (yaw_rate_<0.5)
                    yaw_rate_+=yaw_rate_run_;
                dirty = false;
                break;
            case KEYCODE_S_CAP:
                if (walk_vel_>0)
                    walk_vel_-=run_vel_;
                if (yaw_rate_>0)
                    yaw_rate_-=run_vel_;
                dirty = false;
                break;
            case KEYCODE_A_CAP:
                max_tv2 = walk_vel_;
                speed2=1;
                dirty = true;
                break;
            case KEYCODE_D_CAP:
                max_tv2 = walk_vel_;
                speed2=-1;
                dirty = true;
                break;
            default:
                max_tv1 = walk_vel_;
                max_tv2 = walk_vel_;
                max_rv = yaw_rate_;
                speed1 = 0;
                speed2=0;
                turn = 0;
                dirty = false;
        }
        cmdvel_.linear.x = speed1 * max_tv1;
        cmdvel_.linear.y = speed2 * max_tv2;
        cmdvel_.angular.z = turn * max_rv;
        cmdvel_.linear.z=yaw_rate_;
        pub_.publish(cmdvel_);
    }
}
int main(int argc, char** argv)
{
    ros::init(argc,argv,"telop_key_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    SmartCarKeyboardTeleopNode tbk;

    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}
