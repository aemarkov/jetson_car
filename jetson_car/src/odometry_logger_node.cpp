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

bool   is_log_path;						// Логировать ли одометрию в файл
bool   is_publish_path;     			// Публиковать ли траеткорию, построенную из одометрии
string logs_directory;      			// Выходная папка
const string odom_topic = "/zed/odom";	// Имя топика одометрии 
const string path_topic = "/real_path";	// Имя топика траектории

ofstream trajectory_file;   			// Лог Y(X)
ofstream full_log_file;     			// Лог XYZ RPY Q от времени

ros::Time t0;               			// Начальный момент времени
ros::Publisher path_pub;				// Публишер топика траектории

nav_msgs::Path path;

void read_params(const ros::NodeHandle& nh);
string get_filename(const string& directory, const string& suffix);
void odom_cb(const nav_msgs::Odometry& msg);
bool create_directory_if_not_exists(const string& path);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "zed_logger");
    ros::NodeHandle nh;

    // Получение параметров
    read_params(nh);

    ROS_INFO_STREAM("Path publishing:     " << is_publish_path ? "enabled" : "disabled");
    ROS_INFO_STREAM("Log saving:          " << is_log_path ? "enabled" : "disabled");

    if(is_log_path)
    {
    	if(!create_directory_if_not_exists(logs_directory))
    		exit(0);

	    string trajectory_file_name = get_filename(logs_directory, "_trajectory");
	    string full_log_file_name = get_filename(logs_directory, "_full");
	    trajectory_file.open(trajectory_file_name);
	    full_log_file.open(full_log_file_name);

	    ROS_INFO_STREAM("Trajectory log file: " << trajectory_file_name);
		ROS_INFO_STREAM("Full log file:       " << full_log_file_name);
	    
	    // Заголовки файлов
	    trajectory_file << "X\tY" << endl;
	    full_log_file << "t\tX\tY\tZ\tYaw\tPitch\tRoll\tQ.x\tQ.y\tQ.z\tQ.w" << endl;
	}

	if(!is_log_path && !is_publish_path)
    {
    	ROS_WARN("No publishing, no loggin. Why you run this node?");
    	exit(0);
    }


    // Подписка и публикация
    auto odom_sub = nh.subscribe(odom_topic, 1000, odom_cb);

    if(is_publish_path)
    	path_pub = nh.advertise<nav_msgs::Path>(path_topic, 1);

    t0 = ros::Time::now();   

    ros::spin();
}

void read_params(const ros::NodeHandle& nh)
{		
	nh.param<bool>("is_publish_path", is_publish_path, true);
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

void odom_cb(const nav_msgs::Odometry& msg)
{
	if(is_publish_path)
	{
		path.header = msg.header;
		geometry_msgs::PoseStamped pose;
		pose.header = msg.header;
		pose.pose = msg.pose.pose;
		path.poses.push_back(pose);
		path_pub.publish(path);
	}

	if(is_log_path)
	{		   
		double t = (ros::Time::now() - t0).toSec();

		auto pos = msg.pose.pose.position;
    	auto orientation = msg.pose.pose.orientation;
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