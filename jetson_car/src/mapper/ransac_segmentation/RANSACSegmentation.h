//
// Created by garrus on 11.01.19.
//

#ifndef JETSON_CAR_RANSAC_SEGMENTATION_H
#define JETSON_CAR_RANSAC_SEGMENTATION_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

#include <mapper/GridCoord.h>

/**
 * \brief Find obstacles in point cloud using RANSAC plane segmentation
 *
 * Find ground plane using RANSAC, everything is not plane is obstacles.
 * Represent cloud as nav_msgs::OccupancyGrid
 */
class RANSACSegmentation
{
public:
    /**
     * \brief Create RANSACSegmentation with given cell size and RANSAC threshold
     * @param cell_size Occupancy grid cell size in meteres
     * @param distance_threshold RANSAC distance threshold
     * @param viewer PCLVisualizer for debug purpose. This is not really clear solution
     * @param safe_radius Every obstacle cell will be expanded by this radius
     */
    RANSACSegmentation(float cell_size, float distance_threshold, int cnt_threshold, float safe_radius);

    /**
     * Find obstacles in point cloud and create OccupancyGrid
     * @param[in] cloud source cloud
     * @param[out] grid  resulting occupancy grid
     * @return Is segmentation successfull
     */
    bool calculate(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, nav_msgs::OccupancyGrid& grid);

private:

    const float CELL_SIZE;                           ///< OccupancyGrid cell size in meters
    const float DISTANCE_THRESHOLD;                  ///< RANSAC distance threshold
    const uint8_t SAFE_RADIUS;                       ///< Radius around obstacles marked as dangerous
    const int CNT_THRESHOLD;

    const int8_t OBSTACLE_VALUE = 100;               /// < OccupancyGrid obstacle cells value
    const int8_t NEAR_OBSTACLE_VALUE = 80;           /// < OccupancyGrid cells value in radius around obstacles
    pcl::SACSegmentation<pcl::PointXYZ> seg;         ///< Using for plane segmentation
    tf::TransformBroadcaster _broadcaster;           ///< Using for transform publishing
    tf::TransformListener _listener;                 ///< Using to get transform
    pcl::visualization::PCLVisualizer::Ptr _viewer;

    const std::string WORLD_FRAME = "map";
    const std::string OCCUPANCY_GRID_FRAME = "occupancy_grid";

    /**
     * Trace line from starting point (LIDAR) to the obstacle and mark:
     *  - every cell between lidar and obstacle cell as FREE space
     *  - every cell farther obstacle cell as UNKNOWN space */
    void trace_line(nav_msgs::OccupancyGrid& grid, GridCoord p1, GridCoord p2);


    /**
     * Draw circle around the obstacle to create safe area
     */
    void draw_circle(nav_msgs::OccupancyGrid& grid, GridCoord p, int radius, int8_t value);
    void hline(nav_msgs::OccupancyGrid& grid, int x1, int x2, int y, int8_t value);

    void grid_set(nav_msgs::OccupancyGrid& grid, GridCoord coord, int8_t value);
    int8_t grid_get(nav_msgs::OccupancyGrid& grid, GridCoord coord);

    /**
     * Create transform from WORLD frame to the this OccupancyGrid frame, so grid should match the same
     * position as point cloud */
    bool create_transform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, nav_msgs::OccupancyGrid &grid, const pcl::PointXYZ &min);
};


#endif //JETSON_CAR_RANSAC_SEGMENTATION_H
