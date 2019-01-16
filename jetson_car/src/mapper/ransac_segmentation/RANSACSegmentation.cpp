//
// Created by garrus on 11.01.19.
//

#include "RANSACSegmentation.h"

RANSACSegmentation::RANSACSegmentation(float cell_size, float distance_threshold, float safe_radius, pcl::visualization::PCLVisualizer::Ptr viewer) :
        CELL_SIZE(cell_size),
        DISTANCE_THRESHOLD(distance_threshold),
        SAFE_RADIUS(ceil(safe_radius/cell_size)),
        _viewer(viewer)
{
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold(DISTANCE_THRESHOLD);
}


bool RANSACSegmentation::calculate(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, nav_msgs::OccupancyGrid& grid)
{
    // RANSAC plane estimation
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

     /* TODO: improvements
      *   1) Sometimes RANSAC detect walls or some other planes. Maybe check ModelCoefficients
      *      to select planes which is near to horizontal
      *   2) In rooms lidar can detect ceiling, and this additional points marks as obstacles.
      *      We can use ModelCoefficients to cut all points upper the some plane, parallel to
      *      the ground
      *
      *      Plane:
      *      Ax + By + Cz + D = 0
      *      n(A, B, C) - normal
      *      So, if we change D we can create parallel plane.
      *      IMPORTANT:
      *         1) normal can be directed to the bottom
      *         2) make sure, that |n| = 1
     */

    // debug visualization
    if(_viewer != nullptr)
    {
        if(_viewer->contains("cloud"))
            _viewer->removePointCloud("cloud");
        _viewer->addPointCloud(cloud, "cloud");

        if(_viewer->contains("plane"))
            _viewer->removeShape("plane");
        _viewer->addPlane(coefficients, "plane");
    }

    // Find cloud size and create occupancy grid
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*cloud, min, max);
    uint32_t rows = (uint32_t)ceil((max.y - min.y)/CELL_SIZE);
    uint32_t cols = (uint32_t)ceil((max.x - min.x)/CELL_SIZE);

    grid.info.width = cols;
    grid.info.height = rows;
    grid.info.resolution = CELL_SIZE;
    grid.data.resize(rows*cols, -1);

    std::vector<bool> invert_indexes(cloud->points.size(), true);
    for(int i = 0; i<inliers.indices.size(); i++)
        invert_indexes[inliers.indices[i]] = false;

    int row0 = (int)((0 - min.y)/CELL_SIZE);
    int col0 = (int)((0 - min.x)/CELL_SIZE);
    GridCoord center_coord(row0, col0);

    // Fill occupancy grid with obstacles
    for(int i = 0; i<cloud->points.size(); i++)
    {
        const auto &point = cloud->at(i);
        size_t row = (size_t) ((point.y - min.y) / CELL_SIZE);
        size_t col = (size_t) ((point.x - min.x) / CELL_SIZE);

        if(invert_indexes[i])
        {
            grid_set(grid, GridCoord(row, col), OBSTACLE_VALUE);
        }
    }

    for(size_t row = 0; row<rows; row++)
    {
        trace_line(grid, center_coord, GridCoord(row, 0));
        trace_line(grid, center_coord, GridCoord(row, cols-1));
    }
    for(size_t col = 0; col<cols; col++)
    {
        trace_line(grid, center_coord, GridCoord(0, col));
        trace_line(grid, center_coord, GridCoord(rows-1, col));
    }

    if(SAFE_RADIUS>0)
    {
        for (uint32_t row = 0; row < rows; row++)
        {
            for (uint32_t col = 0; col < cols; col++)
            {
                GridCoord c(row, col);
                if (grid_get(grid, c) == 100)
                    draw_circle(grid, c, 5, NEAR_OBSTACLE_VALUE);
            }
        }
    }

    return create_transform(cloud, grid, min);

}


void RANSACSegmentation::trace_line(nav_msgs::OccupancyGrid &grid, GridCoord p1, GridCoord p2)
{
    /* Bresenham's line algorithm
     *
     * Draw FREE line from p1 to p2 and stop if find OBSTACLE cell
     *
     * NOTE: it's important to go from p1 to p2, do not
     * swap them. DO NOT CHANGE!
     */

    // if dy is more then dx we transpose
    bool is_transposed = false;
    if(abs(p2.x - p1.x) < abs(p2.y - p1.y))
    {
        std::swap(p1.x, p1.y);
        std::swap(p2.x, p2.y);
        is_transposed = true;
    }

    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    int adx = abs(dx);

    int derr = 2*abs(dy);
    int err = 0;
    int y = p1.y;

    int x =  p1.x;
    while(x!=p2.x)
    {
        if (is_transposed)
        {
            if(grid_get(grid, GridCoord(x, y)) > 0)
                return;
            grid_set(grid, GridCoord(x, y), 0);
        }
        else
        {
            if(grid_get(grid, GridCoord(y, x)) > 0)
                return;
            grid_set(grid, GridCoord(y, x), 0);
        }

        err+=derr;
        if(err > adx)
        {
            y += dy>0 ? 1 : -1;
            err -= 2*adx;
        }

        x += dx > 0 ? 1 : -1;
    }
}

void RANSACSegmentation::draw_circle(nav_msgs::OccupancyGrid& grid, GridCoord p, int radius, int8_t value)
{
    int x = 0;
    int y = radius;
    int delta = 1 - 2*radius;
    int error = 0;

    while (y>=0)
    {
        hline(grid, p.x, p.x + x, p.y + y, value);
        hline(grid, p.x, p.x + x, p.y - y, value);
        hline(grid, p.x, p.x - x, p.y + y, value);
        hline(grid, p.x, p.x - x, p.y - y, value);

        error = 2*(delta + y) - 1;
        if(delta < 0 && error <=0)
            delta += 2 * ++x + 1;
        else if(delta > 0 && error > 0)
            delta -= 2* --y + 1;
        else
            delta += 2 * (++x - y--);

    }
}


void RANSACSegmentation::hline(nav_msgs::OccupancyGrid& grid, int x1, int x2, int y, int8_t value)
{
    if(x2 < x1)
        std::swap(x1, x2);

    for(int x = x1; x<= x2; x++)
    {
        GridCoord c(y,x);
        if(y >= 0 && y < grid.info.height && x >= 0 && x < grid.info.width && grid_get(grid, c) <= 0)
            grid_set(grid, c, value);
    }
}

void RANSACSegmentation::grid_set(nav_msgs::OccupancyGrid& grid, GridCoord coord, int8_t value)
{
    grid.data[coord.row * grid.info.width + coord.col] = value;
}

int8_t RANSACSegmentation::grid_get(nav_msgs::OccupancyGrid& grid, GridCoord coord)
{
    return grid.data[coord.row * grid.info.width + coord.col];
}

bool grid_check(nav_msgs::OccupancyGrid &grid, GridCoord coord)
{
    return coord.row >= 0 && coord.row < grid.info.height && coord.col >= 0 && coord.col < grid.info.width;
}


bool RANSACSegmentation::create_transform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, nav_msgs::OccupancyGrid &grid, const pcl::PointXYZ &min)
{
    /* So, we create transform from world frame (usually, map) to occupancy grid frame to
     * keep obstacles on grid in same position in global frame when camera is moving
     *
     * We don't use transform point_cloud -> occupancy_grid, because we should rotate
     * grid in opposite direction to compensate rotation. Also, we need zero Pitch, Roll and
     * z-coordinate.
     *
     * OccupancyGrid origin in (0, 0) cell (left-bottom corner), so we need some math.
     *     OC -  Vector from world origin to the point cloud origin in world frame
     *     CG -  Vector from point cloud origin to the occupancy grid origin in point cloud
     *           frame. Actually, it the (min_x, min_y, 0)
     *     YAW - Point cloud rotation in world frame
     *
     *     CG' - Vector from point cloud origin to the occupancy grid origin in world frame
     *     OG  - Vector from world origin to the occupancy grid origin in world frame
     *           (what we are looking for)
     *
     *     CG' = R*CG,
     *        where R - rotation matrix to rotate (0, 0, -YAW)
     *
     *     RESULT:
     *     OG = OC + CG' = OC + R*CG
     */

    tf::StampedTransform cloud_tf;
    ros::Time cloud_time;
    pcl_conversions::fromPCL(cloud->header.stamp, cloud_time);

    // Get transform Cloud -> World
    try
    {
        _listener.waitForTransform(WORLD_FRAME, cloud->header.frame_id, cloud_time, ros::Duration(0.1));
        _listener.lookupTransform(WORLD_FRAME, cloud->header.frame_id, cloud_time, cloud_tf);
    }
    catch(tf::TransformException ex)
    {
        ROS_WARN_STREAM("No transform available: " << WORLD_FRAME << " -> " << cloud->header.frame_id);
        ROS_WARN_STREAM(ex.what());
        return false;
    }

    double cloud_yaw = getYaw(cloud_tf.getRotation());
    tf::Transform cloud_to_grid_tf;
    tf::Vector3 cloud_origin = cloud_tf.getOrigin();

    tf::Matrix3x3 rot_mat;
    rot_mat.setRPY(0, 0, cloud_yaw);
    cloud_to_grid_tf.setOrigin(rot_mat * tf::Vector3(min.x, min.y, 0) + tf::Vector3(cloud_origin.x(), cloud_origin.y(), 0));
    cloud_to_grid_tf.setRotation(tf::createQuaternionFromYaw(cloud_yaw));

    // use same timestamp for grid message and grid transform
    auto now = ros::Time::now();
    _broadcaster.sendTransform(tf::StampedTransform(cloud_to_grid_tf, now, WORLD_FRAME, OCCUPANCY_GRID_FRAME));
    grid.header.stamp = now;
    grid.header.frame_id = OCCUPANCY_GRID_FRAME;

    return true;
}