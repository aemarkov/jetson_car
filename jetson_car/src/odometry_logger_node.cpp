#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <ros/time.h>

#include <sys/stat.h>

using namespace std;

bool   is_log_path;                         // Логировать ли одометрию в файл
bool   is_log_gps;                          // Логгировать ли GPS в файл
bool   is_publish_path;                     // Публиковать ли траеткорию, построенную из положения
string logs_directory;                      // Выходная папка
const string pos_topic = "/zed/pose";       // Имя топика положения 
const string path_topic = "/real_path";     // Имя топика траектории
const string gps_topic  = "/odometry/gps";  // Имя топика с GPS в том же фрейме, что и path_topic

ofstream trajectory_file;                   // Лог Y(X)
ofstream gps_file;                          // Лог Y(X) по GPS
ofstream full_log_file;                     // Лог XYZ RPY Q от времени

ros::Time t0;                               // Начальный момент времени
ros::Publisher path_pub;                    // Публишер топика траектории

nav_msgs::Path path;

void read_params(const ros::NodeHandle& nh);
string get_filename(const string& directory, const string& suffix);
void pos_cb(const geometry_msgs::PoseStamped& msg);
void gps_cb(const nav_msgs::Odometry& msg);
bool create_directory_if_not_exists(const string& path);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_logger");
    ros::NodeHandle nh;

    // Получение параметров
    read_params(nh);

    ROS_INFO_STREAM("Path publishing:     " << (is_publish_path ? "enabled" : "disabled"));
    ROS_INFO_STREAM("Log saving:          " << (is_log_path ? "enabled" : "disabled"));
    ROS_INFO_STREAM("GPS saving:          " << (is_log_gps ? "enabled" : "disabled"));

    if(is_log_path)
    {
        if(!create_directory_if_not_exists(logs_directory))
            exit(0);

        string trajectory_file_name = get_filename(logs_directory, "_trajectory");
        string gps_trajectory_file_name = get_filename(logs_directory, "_gps");
        string full_log_file_name = get_filename(logs_directory, "_full");

        trajectory_file.open(trajectory_file_name);
        full_log_file.open(full_log_file_name);
        if(is_log_gps)
            gps_file.open(gps_trajectory_file_name);

        ROS_INFO_STREAM("Trajectory log file:     " << trajectory_file_name);
        ROS_INFO_STREAM("GPS Trajectory log file: " << gps_trajectory_file_name);
        ROS_INFO_STREAM("Full log file:           " << full_log_file_name);
        
        // Заголовки файлов
        trajectory_file << "X\tY" << endl;
        full_log_file << "t\tX\tY\tZ\tYaw\tPitch\tRoll\tQ.x\tQ.y\tQ.z\tQ.w" << endl;

        if(is_log_gps)
            gps_file << "X GPS\tY GPS" << endl;
    }

    if(!is_log_path && !is_publish_path && !is_log_gps)
    {
        ROS_WARN("No publishing, no loggin. Why you run this node?");
        exit(0);
    }


    // Подписка и публикация
    auto pos_sub = nh.subscribe(pos_topic, 100, pos_cb);

    ros::Subscriber gps_sub;
    if(is_log_gps)
        gps_sub = nh.subscribe(gps_topic, 5, gps_cb);


    if(is_publish_path)
        path_pub = nh.advertise<nav_msgs::Path>(path_topic, 1);

    t0 = ros::Time::now();   

    ros::spin();
}

void read_params(const ros::NodeHandle& nh)
{       
    nh.param<bool>("is_publish_path", is_publish_path, false);
    nh.param<bool>("is_log_gps", is_log_gps, false);
    is_log_path = nh.getParam("logs_directory", logs_directory);    
}

string get_filename(const string& directory, const string& suffix)
{
    stringstream ss;
    auto t = time(nullptr);
    auto tm = *localtime(&t);
    ss << directory << "/" << "log_" << put_time(&tm, "%Y.%m.%d_%H.%M.%S") << suffix << ".txt";
    return ss.str();
}

bool create_directory_if_not_exists(const string& path)
{
    struct stat st;
    if(stat(path.c_str(), &st)==0)
        return true;

    if(mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)==-1)
    {
        ROS_ERROR_STREAM("Error creating directory. Error code: " << errno);
        return false;
    }    
    
    return true;
}

void pos_cb(const geometry_msgs::PoseStamped& msg)
{
    if(is_publish_path)
    {
        path.header = msg.header;
        path.poses.push_back(msg);
        path_pub.publish(path);
    }

    if(is_log_path)
    {          
        double t = (ros::Time::now() - t0).toSec();

        auto pos = msg.pose.position;
        auto orientation = msg.pose.orientation;
        tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf::Matrix3x3 m(q);
        tfScalar yaw, pitch, roll;
        m.getRPY(yaw, pitch, roll);

        // Сохранение плоского графика траектории X(Y)
        trajectory_file << pos.x << "\t" << pos.y << endl;

        // Сохранение всех данных
        full_log_file << t << "\t" << pos.x << "\t" << pos.y << "\t" << pos.z << "\t" <<
                                      yaw   << "\t" << pitch << "\t" << roll  << "\t" <<
                                      orientation.x   << "\t" << orientation.y   << "\t" << orientation.z << "\t" << orientation.w << endl;

    }
}

void gps_cb(const nav_msgs::Odometry& msg)
{
    if(!is_log_gps)
        return;    

    auto pos = msg.pose.pose.position;
    gps_file << pos.x << "\t" << pos.y << endl;
}