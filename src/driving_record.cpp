#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <fstream>
#include <ctime>

class drivingRecord
{
public:
    drivingRecord();
    ~drivingRecord();

    void set_param();

    void run();

    std::string m_odom_topic_name;

    void callbackWheelLinearAngularVelocity(const geometry_msgs::TwistStampedConstPtr &msg);

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber platform_Wheel_Linear_Angular_Velocity_sub;
    
    double dt = 0.05;

    double m_linear_vel_x;
    double prev_vel_x;
    double curr_vel_x;

    double m_cumulative_distance;

    time_t timer;
    struct tm* t;
};

drivingRecord::drivingRecord()
{
    ROS_INFO("\033[1;32m---->\033[0m Driving_record node is started.");

    set_param();

    platform_Wheel_Linear_Angular_Velocity_sub = nh.subscribe(m_odom_topic_name, 10, &drivingRecord::callbackWheelLinearAngularVelocity, this);

    m_cumulative_distance = 0.0;
    // goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/Docking/goal_pose", 1);
    // tag_detecting_flag_pub = nh.advertise<std_msgs::Bool>("/Docking/tag_detecting_flag",1);
}

drivingRecord::~drivingRecord()
{
    ROS_INFO("\033[1;32m---->\033[0m Driving_record node is destroyed");
}

void drivingRecord::set_param()
{
    nh.param<std::string>("/odom_topic", m_odom_topic_name, "wheelodom");
}

void drivingRecord::callbackWheelLinearAngularVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
    m_linear_vel_x = msg->twist.linear.x;
    if (m_linear_vel_x <0){
        m_linear_vel_x = (-1)*m_linear_vel_x;
    }
    curr_vel_x = m_linear_vel_x;

    // std::cout << curr_vel_x << std::endl;
}


void drivingRecord::run()
{
    if(prev_vel_x != curr_vel_x){
        m_cumulative_distance += curr_vel_x*dt;
    }

    std::cout << m_cumulative_distance << std::endl;

    prev_vel_x = curr_vel_x;

    timer = time(NULL);
    t = localtime(&timer);

    std::cout << t->tm_year+1900 << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "driving_record");

    drivingRecord driving_record;

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        driving_record.run();
        loop_rate.sleep();
    }

    return 0;
}