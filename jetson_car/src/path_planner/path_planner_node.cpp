#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <jetson_car/CarPath.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <iostream>
#include <mutex>

namespace ob = ompl::base;
namespace og = ompl::geometric;

void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

void plan_path();
int get_grid_value(const geometry_msgs::Point& point);
bool is_valid(const ob::State* state);

ros::Publisher path_pub;

std::mutex path_planning_mutex;
nav_msgs::OccupancyGrid::ConstPtr occupancy_grid = nullptr;
geometry_msgs::PoseStamped::ConstPtr goal = nullptr;
geometry_msgs::PoseStamped::ConstPtr pose = nullptr;

std::shared_ptr<ob::SE2StateSpace> space;
std::shared_ptr<og::SimpleSetup> ss;

std::unique_ptr<tf::TransformListener> listener;
const std::string WORLD_FRAME = "map";
const std::string OCCUPANCY_GRID_FRAMW = "occupancy_grid";

int i = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ROS_INFO_STREAM("path_planner_node started");

    listener = std::make_unique<tf::TransformListener>(nh);

    space = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-100);
    bounds.setHigh(100);
    space->setBounds(bounds);

    ss = std::make_shared<og::SimpleSetup>(space);
    ss->setStateValidityChecker(is_valid);

    auto planner = std::make_shared<og::RRTstar>(ss->getSpaceInformation());
    ss->setPlanner(planner);

    auto grid_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/occupancy_grid", 1, grid_callback);
    auto goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 1, goal_callback);
    auto pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/zed/pose", 1, pose_callback);
    path_pub = nh.advertise<nav_msgs::Path>("path", 1);

    ros::spin();
}

void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(path_planning_mutex);
    occupancy_grid = msg;
    plan_path();
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(path_planning_mutex);
    goal = msg;
    plan_path();
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(path_planning_mutex);
    pose = msg;
}

void plan_path()
{
    if(occupancy_grid == nullptr || goal == nullptr)
        return;

    /*geometry_msgs::PointStamped in, out;
    in.header.frame_id = WORLD_FRAME;
    in.header.stamp = occupancy_grid->header.stamp;
    in.point = goal->pose.position;
    listener->transformPoint(OCCUPANCY_GRID_FRAMW, in, out);
    int8_t value = get_grid_value(out.point);
    if(value > 0)
        ROS_INFO("Obstacle");
    else if(value == 0)
        ROS_INFO("Free");
    else
        ROS_INFO("Unknown");*/

    ob::ScopedState<ob::SE2StateSpace> start_state(space);
    start_state->setXY(pose->pose.position.x, pose->pose.position.y);
    start_state->setYaw(tf::getYaw(pose->pose.orientation));

    ob::ScopedState<ob::SE2StateSpace> goal_state(space);
    goal_state->setXY(goal->pose.position.x, goal->pose.position.y);
    goal_state->setYaw(tf::getYaw(goal->pose.orientation));

    i = 0;
    ss->clear();
    ss->setStartAndGoalStates(start_state, goal_state);
    auto solved = ss->solve(0.5);
    ROS_INFO_STREAM("is_valid called "  << i);
    if(solved)
    {
        ss->simplifySolution();
        auto states = ss->getSolutionPath().getStates();

        nav_msgs::Path path;
        path.header.frame_id = "map";

        for(const ob::State* state: states)
        {
            auto* se2state = state->as<ob::SE2StateSpace::StateType>();
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = se2state->getX();
            pose.pose.position.y = se2state->getY();
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(se2state->getYaw());
            path.poses.push_back(pose);
        }

        path_pub.publish(path);
    }
}

int get_grid_value(const geometry_msgs::Point& point)
{
    int64_t row = point.y / occupancy_grid->info.resolution;
    int64_t col = point.x / occupancy_grid->info.resolution;
    if(row < 0 || row >= occupancy_grid->info.height || col < 0 || col >= occupancy_grid->info.width)
        return 0;

    return occupancy_grid->data[row * occupancy_grid->info.width + col];
}

bool is_valid(const ob::State* state)
{
    i++;
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
    double x = se2state->getX();
    double y = se2state->getY();
    double yaw = se2state->getYaw();

    geometry_msgs::PointStamped in, out;
    in.header.frame_id = WORLD_FRAME;
    in.header.stamp = occupancy_grid->header.stamp;
    in.point.x = x;
    in.point.y = y;
    in.point.z = 0;
    listener->transformPoint(OCCUPANCY_GRID_FRAMW, in, out);


    return get_grid_value(out.point) <= 0;
}