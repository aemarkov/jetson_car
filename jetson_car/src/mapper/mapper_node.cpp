/*
 * This ros node collect data from multiple sources and create
 * united digital representation of the environment
 *
 * NOW SUPPORTED:
 *  - Point cloud from velodyne -> obstacles OccupancyGrid
 */

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <memory>
#include <pcl/visualization/pcl_visualizer.h>

#include "ransac_segmentation/RANSACSegmentation.h"
#include <jetson_car/DigitalMap.h>

//Uncomment to enable debug visualization
//#define DEBUG_VIEW

void pcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

ros::Publisher occupancy_grid_publisher;
std::unique_ptr<RANSACSegmentation> segmentation;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapper_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Read parameters
    float cell_size, distance_threshold, safe_radius;
    int cnt_threshold;

    private_nh.param<float>("cell_size", cell_size, 0.25);
    private_nh.param<float>("distance_threshold", distance_threshold, 0.1);
    private_nh.param<int>("cnt_threshold", cnt_threshold, 5);
    private_nh.param<float>("safe_radius", safe_radius, 0.5);

    ROS_INFO("Mapper node started");
    ROS_INFO_STREAM("cell_size:          " << cell_size);
    ROS_INFO_STREAM("distance_threshold: " << distance_threshold);
    ROS_INFO_STREAM("cnt_threshold:      " << cnt_threshold);
    ROS_INFO_STREAM("safe_radius:        " << safe_radius);

    pcl::visualization::PCLVisualizer::Ptr viewer = nullptr;
#ifdef DEBUG_VIEW
    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D viewer");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
#endif

    segmentation = std::make_unique<RANSACSegmentation>(cell_size, distance_threshold, cnt_threshold, safe_radius);
    auto cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("point_cloud", 1, pcloud_callback);
    occupancy_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);

#if defined(DEBUG_VIEW)
    ros::Duration t(0.05);
    while(ros::ok())
    {
        ros::spinOnce();
        viewer->spinOnce();
        t.sleep();
    }
#else
    ros::spin();
#endif

    return 0;
}

void pcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg<pcl::PointXYZ>(*msg, *cloud);
    nav_msgs::OccupancyGrid occupancy_grid;
    if(segmentation->calculate(cloud, occupancy_grid))
        occupancy_grid_publisher.publish(occupancy_grid);
}