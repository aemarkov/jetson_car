#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <jetson_car/CarPath.h>

#include <iostream>
#include <mutex>
#include <queue>
#include <map>
#include <list>


struct GridCoord
{
    union {
        struct {
            int row, col;
        };
        struct {
            int y, x;
        };
        struct {
            int height, width;
        };
    };

    GridCoord(){}
    GridCoord(int row, int col)
    {
        this->row = row;
        this->col = col;
    }
};

struct ObstacleOnPath
{
    size_t closest_point;
    bool has_in, has_out;
    size_t out_index;
    GridCoord out_coord;
};


struct AStarState
{
    GridCoord coord;
    AStarState* parent = nullptr;
    float g = std::numeric_limits<float>::max();  // Path cost from start to current
    float h = std::numeric_limits<float>::max();  // Heuristic from current to end

    float f()const {return g+h;}

    AStarState(){}
    AStarState(GridCoord coord){this->coord = coord;}
    AStarState(GridCoord coord, float g, float h, AStarState* parent = nullptr)
    {
        this->coord = coord;
        this->g = g;
        this->h = h;
        this->parent = parent;
    }
};

void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void path_callback(const nav_msgs::Path::ConstPtr& msg);

void plan_path();
ObstacleOnPath find_obstacle(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& pose, double timeout, double speed);
size_t find_closest_point_on_path(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& pose);

geometry_msgs::PointStamped world_point_to_grid(const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point point);
bool grid_point_to_cell(const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point p, GridCoord& coord);
int8_t get_grid_value(const nav_msgs::OccupancyGrid& grid, GridCoord coord);

nav_msgs::OccupancyGrid::ConstPtr grid = nullptr;
//geometry_msgs::PoseStamped::ConstPtr goal = nullptr;
geometry_msgs::PoseStamped::ConstPtr pose  = nullptr;
nav_msgs::Path::ConstPtr reference_path  = nullptr;

ros::Publisher path_pub;
std::unique_ptr<tf::TransformListener> listener;
std::mutex path_planning_mutex;
const std::string WORLD_FRAME = "map";
const std::string OCCUPANCY_GRID_FRAME = "occupancy_grid";

const double REPLANE_INTERVAL = 0.5;
const double LONGITUDINAL_SPEED = 3.0;


bool is_wtf = true;
geometry_msgs::Point end_point;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ROS_INFO_STREAM("path_planner_node started");

    listener = std::make_unique<tf::TransformListener>(nh);
    auto grid_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/occupancy_grid", 1, grid_callback);
    //auto goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, goal_callback);
    auto pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/zed/pose", 1, pose_callback);
    auto path_sub = nh.subscribe<nav_msgs::Path>("/reference_path", 1, path_callback);
    path_pub = nh.advertise<nav_msgs::Path>("/local_path", 1);


    ros::Rate rate(1.0/REPLANE_INTERVAL);
    while(ros::ok())
    {
        ros::spinOnce();
        if(is_wtf)
            plan_path();

        rate.sleep();
    }
}


void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    //std::lock_guard<std::mutex> guard(path_planning_mutex);
    grid = msg;
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //ROS_INFO_STREAM((int)get_grid_value(*grid, msg->pose.position));
    pose = msg;
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //std::lock_guard<std::mutex> guard(path_planning_mutex);
    float dist = sqrt(pow(msg->pose.position.x - end_point.x, 2) + pow(msg->pose.position.y - end_point.y, 2));
    if(!is_wtf)
        ROS_INFO_STREAM("Dist to path end: " << dist);

    if(dist < 0.4 && !is_wtf)
    {
        is_wtf = true;
        ROS_INFO("Path unlocked");
    }
    pose = msg;
}

void path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    //std::lock_guard<std::mutex> guard(path_planning_mutex);
    reference_path = msg;
}

void a_star_path(const nav_msgs::OccupancyGrid& grid, AStarState* state, std::vector<geometry_msgs::PoseStamped>& path)
{
    geometry_msgs::PointStamped in, out;
    in.header.frame_id = OCCUPANCY_GRID_FRAME;
    in.header.stamp = grid.header.stamp;

    while(state != nullptr)
    {
        in.point.x = (state->coord.col + 0.5) * grid.info.resolution;
        in.point.y = (state->coord.row + 0.5)* grid.info.resolution;

        listener->transformPoint(WORLD_FRAME, in, out);

        geometry_msgs::PoseStamped pose;
        pose.pose.position = out.point;
        pose.header.frame_id = WORLD_FRAME;

        path.push_back(pose);
        state = state->parent;
    }

    std::reverse(std::begin(path), std::end(path));

}

float a_star_dist(const nav_msgs::OccupancyGrid& grid, const GridCoord& p1, const GridCoord& p2)
{
    float x1 = p1.col * grid.info.resolution;
    float y1 = p1.row * grid.info.resolution;
    float x2 = p2.col * grid.info.resolution;
    float y2 = p2.row * grid.info.resolution;

    return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
}

AStarState* a_star_frontier_pop(std::map<size_t, AStarState*>& frontier)
{
    float f_min = std::numeric_limits<float>::max();
    std::map<size_t, AStarState*>::iterator min_it;
    for(auto it = frontier.begin(); it!=frontier.end(); ++it)
    {
        if(it->second->f() < f_min)
        {
            f_min = it->second->f();
            min_it = it;
        }
    }

    AStarState* state = min_it->second;
    frontier.erase(min_it);
    return state;
}

bool a_star(const nav_msgs::OccupancyGrid& grid, GridCoord from, GridCoord to,  std::vector<geometry_msgs::PoseStamped>& path)
{
    int width = grid.info.width;
    int height = grid.info.height;

    std::map<size_t, AStarState*> frontier;
    std::list<AStarState*> closed;
    std::vector<bool> visited(width*height, false);

    size_t from_i = from.row * grid.info.width + from.col;
    frontier.insert(std::make_pair(from_i, new AStarState(from, 0, a_star_dist(grid, from, to))));
    //visited[from.row * width + from.col] = true;


    while(!frontier.empty())
    {
        AStarState* current = a_star_frontier_pop(frontier);

        visited[current->coord.row * width + current->coord.col] = true;
        closed.push_back(current);

        if(current->coord.row == to.row && current->coord.col == to.col)
        {
            a_star_path(grid, current, path);
            auto it = closed.begin();
            while(it!=closed.end())
            {
                delete *it;
                it = closed.erase(it);
            }
            return true;
        }

        for(int i = -1; i<=1; i++)
        {
            for(int j = -1; j<=1; j++)
            {
                int row = current->coord.row + i;
                int col = current->coord.col + j;
                if(row < 0 || row > height || col < 0 || col > width || visited[row * width + col])
                    continue;

                if(grid.data[row * grid.info.width + col] > 0)
                    continue;

                GridCoord c(row, col);
                size_t c_i = row*grid.info.width + col;
                float g = current->g + a_star_dist(grid, current->coord, c);

                if(frontier.find(c_i)!=frontier.end())
                {
                    AStarState* neighbour = frontier[c_i];
                    if(neighbour->g > g)
                    {
                        neighbour->g = g;
                        neighbour->parent = current;
                    }
                }
                else
                {
                    frontier.insert(std::make_pair(c_i, new AStarState(c, g, a_star_dist(grid, c, to), current)));
                }
            }
        }
    }

    return false;
}

void interpolate_path(const std::vector<geometry_msgs::PoseStamped>& sub_path, std::vector<geometry_msgs::PoseStamped>& interpolated, int factor)
{
    int N = sub_path.size();
    int N1 = N + (N-1)*(factor-1);
    interpolated.resize(N1);
    for(int i = 0; i<sub_path.size()-1; i++)
    {
        auto point = sub_path[i];
        auto p1 = sub_path[i].pose.position;
        auto p2 = sub_path[i+1].pose.position;
        float x_step = (p2.x - p1.x)/factor;
        float y_step = (p2.y - p1.y)/factor;


        interpolated.push_back(point);
        for(int j = 0; j<factor-1; j++)
        {
            point.pose.position.x+=x_step;
            point.pose.position.y+=y_step;
            interpolated.push_back(point);
        }
    }

    interpolated.push_back(sub_path[N-1]);
}

void plan_path()
{
    //std::lock_guard<std::mutex> guard(path_planning_mutex);
    std::vector<geometry_msgs::PoseStamped> sub_path_interpolated();

    if(reference_path == nullptr || pose == nullptr || grid == nullptr)
        return;

    nav_msgs::Path path;
    path.header.frame_id = WORLD_FRAME;
    path.header.stamp = ros::Time::now();

    auto obstacle = find_obstacle(*reference_path, *pose, REPLANE_INTERVAL, LONGITUDINAL_SPEED);
    /*ROS_INFO("1");
    ROS_INFO_STREAM("out_index: " << obstacle.out_index);
    ROS_INFO_STREAM("CP: " << obstacle.closest_point);
    ROS_INFO_STREAM("len: " << reference_path->poses.size());
    ROS_INFO_STREAM("In, out" << obstacle.has_in << " "<<obstacle.has_out);*/

    if(!obstacle.has_in && !obstacle.has_out)
    {
        ROS_INFO("No obstacle");
        //ROS_INFO_STREAM(obstacle.closest_point << " " << obstacle.out_index);

        for(int i = obstacle.closest_point; i<=obstacle.out_index; i++)
            path.poses.push_back(reference_path->poses[i]);
    }
    else if(obstacle.has_in && !obstacle.has_out)
    {
        ROS_WARN("No path");
    }
    else
    {
        //auto p = reference_path->poses[obstacle.closest_point].pose.position;
        //ROS_INFO("3");
        GridCoord p_coord;
        if(grid_point_to_cell(*grid, world_point_to_grid(*grid, pose->pose.position).point, p_coord))
        {
            //ROS_INFO("4");
            std::vector<geometry_msgs::PoseStamped> sub_path;
            if(a_star(*grid, p_coord, obstacle.out_coord, sub_path))
            {

                /*for(int i = obstacle.closest_point; i<=obstacle.in_index; i++)
                    path.poses.push_back(reference_path->poses[i]);*/

                std::vector<geometry_msgs::PoseStamped> sub_path_interpolated;
                interpolate_path(sub_path,  sub_path_interpolated, 3);
                ROS_INFO_STREAM("Path: " << sub_path.size() << " " <<sub_path_interpolated.size());

                for(auto& p: sub_path_interpolated)
                    path.poses.push_back(p);

                int cnt = obstacle.out_index+10;
                if(cnt > reference_path->poses.size())
                    cnt = reference_path->poses.size();

                for(int i = obstacle.out_index; i<cnt; i++)
                    path.poses.push_back(reference_path->poses[i]);

                end_point = path.poses[path.poses.size()-1].pose.position;
                is_wtf = false;
                ROS_INFO("Moving around obstacle");
                ROS_INFO("Path locked");
            }
            else
            {
                ROS_WARN("No path");
            }
        }
        else
        {
            ROS_WARN("Wtf");
        }
    }

    path_pub.publish(path);
}

ObstacleOnPath find_obstacle(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& pose, double timeout, double speed)
{
    // Assume points is close to each other, so we shouldn't check collision between points

    size_t index = find_closest_point_on_path(path, pose);
    ObstacleOnPath obstacle;
    obstacle.closest_point = index;
    bool find_in = true;

    double t = 0;
    while(index < path.poses.size() - 1 && (t < timeout || !find_in))
    {
        auto p = path.poses[index].pose.position;
        auto pnext = path.poses[index+1].pose.position;
        GridCoord p_coord, pnext_coord;
        int8_t p_value, pnext_value;
        if(grid_point_to_cell(*grid, world_point_to_grid(*grid, p).point, p_coord))
            p_value = get_grid_value(*grid, p_coord);
        else
            p_value = -1;

        if(grid_point_to_cell(*grid, world_point_to_grid(*grid, p).point, pnext_coord))
            pnext_value = get_grid_value(*grid, pnext_coord);
        else
            pnext_value = -1;


        obstacle.out_index = index;

        if(find_in && pnext_value > 0)
        {
            find_in = false;
            obstacle.has_in = true;
        }
        else if(!find_in && p_value <= 0)
        {
            obstacle.out_coord = p_coord;
            obstacle.has_out = true;
            break;
        }

        double dist = sqrt(pow(p.x - pnext.x, 2) + pow(p.y - pnext.y, 2));
        t += dist / speed;
        index++;
    }

    return  obstacle;
}

size_t find_closest_point_on_path(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& pose)
{
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_point_index = 0;
    for(size_t i = 0; i<path.poses.size(); i++)
    {
        double dist = pow(path.poses[i].pose.position.x - pose.pose.position.x, 2) + pow(path.poses[i].pose.position.y - pose.pose.position.y, 2);
        if(dist < min_dist)
        {
            min_dist = dist;
            closest_point_index = i;
        }
    }

    return closest_point_index;
}


geometry_msgs::PointStamped world_point_to_grid(const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point point)
{
    geometry_msgs::PointStamped in, out;
    in.header.frame_id = WORLD_FRAME;
    in.header.stamp = grid.header.stamp;
    in.point = point;
    listener->transformPoint(OCCUPANCY_GRID_FRAME, in, out);
    return out;
}

bool grid_point_to_cell(const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point p, GridCoord& coord)
{
    int row = p.y / grid.info.resolution;
    int col  = p.x / grid.info.resolution;

    if(row < 0 || row >= grid.info.height || col < 0 || col >= grid.info.width)
        return false;

    coord.row = row;
    coord.col = col;

    return true;
}

int8_t get_grid_value(const nav_msgs::OccupancyGrid& grid, GridCoord coord)
{
    return grid.data[coord.row * grid.info.width + coord.col];
}
