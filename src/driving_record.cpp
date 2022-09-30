#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
#include <vector>

class drivingRecord
{
public:
    drivingRecord();
    ~drivingRecord();

    void set_param();

    void run();

    std::string m_odom_topic_name;
    std::string save_path;

    void callbackWheelLinearAngularVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
    std::int32_t fileSize(std::string fileName); // file size reture function

    bool test_bool, test_bool_2;    

    std::string buffer;
    std::string compare_buffer;
    std::string write_input;
    std::vector<std::string> dictionary;
    int count;

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber platform_Wheel_Linear_Angular_Velocity_sub;
    ros::Publisher odom_1Day_pub;
    ros::Publisher odom_Total_pub;
    ros::Publisher time_1Day_pub;
    ros::Publisher time_Total_pub;

    std_msgs::Float32 odom_1Day_msg;
    std_msgs::Float32 odom_Total_msg;
    std_msgs::Float32 time_1Day_msg;
    std_msgs::Float32 time_Total_msg;
    
    float dt = 0.05;

    float m_linear_vel_x;
    float m_prev_odom_time;
    float m_curr_odom_time;

    float m_cumulative_distance_1day;
    float m_cumulative_distance_total;
    float m_cumulative_time_1day;
    float m_cumulative_time_total;

    time_t timer;
    struct tm* t;

    int time_sec;			/* Seconds.	[0-60] (1 leap second) */
    int time_min;			/* Minutes.	[0-59] */
    int time_hour;			/* Hours.	[0-23] */
    int time_mday;			/* Day.		[1-31] */
    int time_mon;			/* Month.	[0-11] */
    int time_year;			/* Year	- 1900.  */

    std::fstream recordingFile;
    
    int compare_time_year;
    int compare_time_mon;
    int compare_time_mday;

    bool is_first_opened;
};

drivingRecord::drivingRecord()
{
    ROS_INFO("\033[1;32m---->\033[0m Driving_record node is started.");

    set_param();

    platform_Wheel_Linear_Angular_Velocity_sub = nh.subscribe(m_odom_topic_name, 10, &drivingRecord::callbackWheelLinearAngularVelocity, this);

    odom_1Day_pub = nh.advertise<std_msgs::Float32>("/Odom_1Day",1);
    odom_Total_pub = nh.advertise<std_msgs::Float32>("/Odom_Total",1);
    time_1Day_pub = nh.advertise<std_msgs::Float32>("/Time_1Day",1);
    time_Total_pub = nh.advertise<std_msgs::Float32>("/Time_Total",1);

    is_first_opened = true;

    buffer = "";
    compare_buffer = "";

    count = 0;

    m_cumulative_distance_1day = 0.0;
    m_cumulative_time_1day = 0.0;

    m_prev_odom_time = 0.0;
    m_curr_odom_time = 0.0;
    
    // If there is no file to record, it is created. 
    recordingFile.open(save_path, std::ios::app);
        
    // if the file size is zero (empty file), insert item name
    if(recordingFile.is_open() == true){
        if(fileSize(save_path) == 0){
            recordingFile << "Year,Mon,Day,Odom_1Day,Odom_Total,Time_1Day,Time_Total,#" << std::endl;
            std::cout << "The log file was first created." << std::endl;
        }
    }
    recordingFile.close();
}

drivingRecord::~drivingRecord()
{
    ROS_INFO("\033[1;32m---->\033[0m Driving_record node is destroyed");
    if( recordingFile.is_open() ){
        recordingFile.close();
        ROS_INFO("\033[1;32m---->\033[0m Recording file is closed.");
    }
}

void drivingRecord::set_param()
{
    nh.param<std::string>("/odom_topic", m_odom_topic_name, "wheelodom");
    nh.param<std::string>("/save_path", save_path, "driving_record.csv");
}


int32_t drivingRecord::fileSize(std::string fileName)
{
    struct stat _stat;
    int32_t rc = stat(fileName.c_str(), &_stat);
    std::int32_t returnValue;
    if(rc == 0)
    {
        returnValue = _stat.st_size;
    }
    else
    {
        returnValue = -1;
    }

    return returnValue;
}

void drivingRecord::callbackWheelLinearAngularVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
    m_linear_vel_x = (float)msg->twist.linear.x;
    if (m_linear_vel_x <0){
        m_linear_vel_x = (-1.0)*m_linear_vel_x;
    }
    m_curr_odom_time = ros::Time::now().toSec();
    // curr_vel_x = m_linear_vel_x;
}


void drivingRecord::run()
{
    // if(prev_vel_x != curr_vel_x)
    if(m_prev_odom_time != m_curr_odom_time)
    {
        m_cumulative_distance_1day += m_linear_vel_x*dt;
        m_cumulative_time_1day += dt;
    }
    else
    {
        m_cumulative_distance_1day = 0.0;
    }

    //////////// for test //////////////
    // m_cumulative_distance_1day = 12.54;
    // m_cumulative_distance_1day = 0.0;
    // m_cumulative_time_1day = 375.45;
    ////////////////////////////////////
    
    m_prev_odom_time = m_curr_odom_time;
    // prev_vel_x = curr_vel_x;

    timer = time(NULL);
    t = localtime(&timer);

    time_sec = t->tm_sec;
    time_min = t->tm_min;
    time_hour = t->tm_hour;
    time_mday = t->tm_mday;
    time_mon = t->tm_mon + 1;
    time_year = t->tm_year + 1900;

    // Save once every 2 seconds
    if(count%40 == 0){
        // open the file with read & write mode.
        recordingFile.open(save_path, std::ios::out | std::ios::in);

        // For detect first line end char '#'
        std::string str_temp = "#";
        recordingFile.seekg(55, std::ios::beg);
        
        // read one line
        getline(recordingFile, buffer);
        std::cout << "buffer: " << buffer << std::endl;

        if(buffer.compare(str_temp) == 0)
        {
            // If there is only the first line
            if(fileSize(save_path) == 57)
            {
                m_cumulative_distance_total = m_cumulative_distance_1day;
                m_cumulative_time_total = m_cumulative_time_1day;

                recordingFile.precision(2);
                recordingFile.setf(std::ios::fixed);
                recordingFile << time_year << "," 
                            << time_mon << "," 
                            << time_mday << "," 
                            << m_cumulative_distance_1day << "," 
                            << m_cumulative_distance_total << "," 
                            << m_cumulative_time_1day << "," 
                            << m_cumulative_time_total << std::endl;
                std::cout << "first write." << std::endl;
            }
            // If records of different dates already exist
            else 
            {
                std::cout << "else" << std::endl;
                dictionary.clear();
                
                // read line by line to the end of the file
                // Save the previous line to the compare buffer
                while(!recordingFile.eof())
                {
                    compare_buffer = buffer;
                    getline(recordingFile, buffer);
                }

                // Separate by comma and store in vector
                char temp[255];
                strcpy(temp, compare_buffer.c_str());
                char *fline = strtok(temp, ",");
                while (fline != NULL)
                {
                    dictionary.push_back(std::string(fline));
                    fline = strtok(NULL, ",");
                }

                // for(auto i: dictionary){
                //     std::cout << i << std::endl;
                // }

                compare_time_mday = std::stoi(dictionary.at(2));
                compare_time_mon = std::stoi(dictionary.at(1));
                compare_time_year = std::stoi(dictionary.at(0));
                
                // Compare the date in the last line with today's date
                // If today's date is a new day
                if((compare_time_mday < time_mday) || (compare_time_mon < time_mon) || (compare_time_year < time_year))
                {
                    if(buffer.empty())
                    {
                        recordingFile.clear();
                        recordingFile.seekp(0, std::ios::end);
                        recordingFile.seekg(0, std::ios::end);
                    }

                    m_cumulative_distance_total = std::stof(dictionary.at(4)) + m_cumulative_distance_1day;
                    m_cumulative_time_total = std::stof(dictionary.at(6)) + m_cumulative_time_1day;

                    recordingFile.precision(2);
                    recordingFile.setf(std::ios::fixed);
                    recordingFile << time_year << "," 
                                << time_mon << "," 
                                << time_mday << "," 
                                << m_cumulative_distance_1day << "," 
                                << m_cumulative_distance_total << "," 
                                << m_cumulative_time_1day << "," 
                                << m_cumulative_time_total << std::endl;
                    
                    std::cout << "new day added" << std::endl;
                }
                // If you drive again today for the second time or more
                else if((compare_time_year == time_year) && (compare_time_mon == time_mon) && (compare_time_mday == time_mday))
                {

                    if(buffer.empty())
                    {
                        recordingFile.clear();
                        recordingFile.seekp(-(compare_buffer.length()+1), std::ios::end);
                        recordingFile.seekg(-(compare_buffer.length()+1), std::ios::end);
                    }

                    m_cumulative_distance_1day = std::stof(dictionary.at(3)) + m_cumulative_distance_1day;
                    m_cumulative_time_1day = std::stof(dictionary.at(5)) + m_cumulative_time_1day;

                    m_cumulative_distance_total = std::stof(dictionary.at(4)) + m_cumulative_distance_1day;
                    m_cumulative_time_total = std::stof(dictionary.at(6)) + m_cumulative_time_1day;

                    recordingFile.precision(2);
                    recordingFile.setf(std::ios::fixed);
                    recordingFile << time_year << "," 
                                << time_mon << "," 
                                << time_mday << "," 
                                << m_cumulative_distance_1day << "," 
                                << m_cumulative_distance_total << "," 
                                << m_cumulative_time_1day << "," 
                                << m_cumulative_time_total << std::endl;
                    
                    std::cout << "today launch again" << std::endl;
                }
            }
        }
        recordingFile.close();

        count = 0;
    }

    count ++;
    
    odom_1Day_msg.data = m_cumulative_distance_1day;
    odom_Total_msg.data = m_cumulative_distance_total;
    time_1Day_msg.data = m_cumulative_time_1day;
    time_Total_msg.data = m_cumulative_time_total;

    odom_1Day_pub.publish(odom_1Day_msg);
    odom_Total_pub.publish(odom_Total_msg);
    time_1Day_pub.publish(time_1Day_msg);
    time_Total_pub.publish(time_Total_msg);
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