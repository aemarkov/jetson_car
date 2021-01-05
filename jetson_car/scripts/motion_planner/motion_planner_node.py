#!/usr/bin/env python
#coding=utf-8

import math
import sys
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, Pose, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

import quintic
import frenet
from rviz_helpers import LinesHelper
import msg_helpers
from trajectory import Trajectory

WORLD_FRAME = 'map'
OCCUPANCY_GRID_FRAME = 'occupancy_grid'

REPLANE_PERIOD = 0.5         # Replane period in seconds
HORIZONT = 3                 # Planning horizont - arc length of the reference path for which planning will be occured
ROAD_WIDTH = 2               # Road width - width of the allowable area  around the reference path

D_MIN  = -ROAD_WIDTH/2.0     # Minimum of the lateral variants
D_MAX  =  ROAD_WIDTH/2.0     # Maximum of the lateral variants
D_STEP =  0.2                # Step between lateral variants

S_MIN  =  1
S_MAX  =  HORIZONT
S_STEP =  0.5

S_CALC_STEP = 0.1            # Curves interpolation step

Kj = 0#0.0005
Kt = 0.1
Kd = 5.0

grid = None
pose = None
reference_path = None

def grid_callback(msg):
    global grid
    grid = msg
    #rospy.loginfo('grid')


def path_callback(msg):
    global reference_path
    reference_path = msg
    #rospy.loginfo('path')

############################################################################

def rot_matrix(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s],
                     [s,  c]])

def rot_vector(vec, angle):
    return np.matmul(rot_matrix(angle), vec)

def vec_of_angle(angle):
    return rot_vector(np.array([1,0]), angle)

# Find index of the point in path closest to given
def find_closest_point_index(path, position, start_index = 0):
    min_index = -1
    min_dist = sys.float_info.max

    for i in range(start_index, len(path.poses)):
        dist = (path.poses[i].pose.position.x - position.x)**2 + (path.poses[i].pose.position.y - position.y)**2
        if dist < min_dist:
            min_dist = dist
            min_index = i

    return min_index

# Get the part of the path from start index and certain length
def get_subpath(path, start_index, length):
    s = 0
    i = start_index

    while i < len(path.poses)-1 and s < length:
        p1 = path.poses[i].pose.position
        p2 = path.poses[i+1].pose.position
        dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        s+=dist
        #rospy.loginfo('%0.2f, %0.2f' % (p1.x, p2.x))
        i+=1

    if i >= len(path.poses):
        return path.poses[start_index:]

    return path.poses[start_index:i+1] # include right

# Combine lateral ang longtitudial trajectories
# For now lon trajectory is just central line, so combine
# trajectory with itself x values
def combine_trajectory(trajectory):
    arr = np.zeros((trajectory.len(), 2))
    for i in range(trajectory.len()):
        arr[i] = [trajectory.x[i], trajectory.y[i]]
    return arr

# Integrate jerk (d3y/dx^3) for a given trajectory
def integrate_jerk(trajectory):
    J = 0
    for i in range(trajectory.len()-1):
        dj = (trajectory.ddy[i+1]-trajectory.ddy[i])/(trajectory.x[i+1]-trajectory.x[i])
        J += dj*dj

    return J

# Get value of the grid's cell in the certain point
# point in world frame
def get_grid_value(grid, point):

    # Transform from world frame to the grid frame
    point_stamped = PointStamped()
    point_stamped.header.stamp = rospy.Time(0)
    point_stamped.header.frame_id = WORLD_FRAME
    point_stamped.point = point
    point_grid = listener.transformPoint(OCCUPANCY_GRID_FRAME, point_stamped)

    # Get cell
    row = int(point_grid.point.y / grid.info.resolution)
    col = int(point_grid.point.x / grid.info.resolution)

    if row < 0 or row >= grid.info.height or col < 0 or col >= grid.info.width:
        return -1

    return grid.data[row * grid.info.width + col]


# Check all path's points not in obstacle cells
def check_path_clear(grid, trajectory):
    for p in trajectory:
        if get_grid_value(grid, msg_helpers.array_to_point(p)) > 0:
            return False
    return True

def draw_road(np_subpath):
    n = 50
    fp_c = np.zeros((n, 2))
    fp_min = np.zeros((n, 2))
    fp_max = np.zeros((n, 2))
    for i, s in enumerate(np.linspace(0, HORIZONT, n)):
        fp_min[i] = [s, D_MIN]
        fp_c[i] = [s, 0]
        fp_max[i] = [s, D_MAX]


    fp_min = frenet.path_to_global(fp_min, np_subpath)
    fp_c = frenet.path_to_global(fp_c, np_subpath)
    fp_max = frenet.path_to_global(fp_max, np_subpath)

    road_borders_helper.lines([fp_min, fp_c, fp_max])

def plan_path():
    if grid == None or pose == None or reference_path == None:
        return

    start_index = find_closest_point_index(reference_path, pose.pose.position)
    subpath = get_subpath(reference_path, start_index, HORIZONT)
    np_subpath = msg_helpers.path_poses_to_array(subpath)

    # Convert current position to Frenet Frame
    orientation =  tf.transformations.euler_from_quaternion(msg_helpers.quaterion_to_array(pose.pose.orientation))[2]
    pos = msg_helpers.point_to_array(pose.pose.position)
    speed = vec_of_angle(orientation)
    frenet_frame = frenet.FrenetFrame(0, np_subpath[0], np_subpath[1])
    pos_frenet = frenet_frame.point_to(pos)
    speed_frenet = frenet_frame.vector_to(speed)

    d1 = pos_frenet[1]     # initial lat position in frenet frame
    dd1 = speed_frenet[1]  # initial lat speed in frenet frame

    trajectories = []
    min_cd = sys.float_info.max
    optimal_trajectory = None

    for si in np.arange(S_MIN, S_MAX+S_STEP, S_STEP):
        #print(si)
        ss = np.arange(0, si, S_CALC_STEP)
        for di in np.arange(D_MIN, D_MAX+D_STEP, D_STEP):
            d2 = (0, 0, S_MAX)
            lat_coefs = quintic.calc_coefs((d1, dd1, 0), (di, 0, 0), si)
            lat_trajectory = Trajectory(ss, *quintic.interpolate(lat_coefs, ss))

            frenet_trajectory = combine_trajectory(lat_trajectory)
            global_trajectory = frenet.path_to_global(frenet_trajectory, np_subpath)
            trajectories.append(global_trajectory)

            if not check_path_clear(grid, global_trajectory):
                continue

            Cd = Kj*integrate_jerk(lat_trajectory) + Kt*lat_trajectory.x[-1] + Kd*lat_trajectory.y[-1]**2
            if Cd < min_cd:
                optimal_trajectory = global_trajectory
                min_cd = Cd

    draw_road(np_subpath)
    trajectories_helper.lines(trajectories)

    local_path = Path()
    local_path.header.frame_id = WORLD_FRAME

    if optimal_trajectory is not None:
        optimal_trajectory_helper.lines([optimal_trajectory])

        for np_p in optimal_trajectory:
            p = PoseStamped()
            p.pose.position = msg_helpers.array_to_point(np_p)
            local_path.poses.append(p)
    else:
        optimal_trajectory_helper.lines([np.zeros((1,2))])
        rospy.logwarn('No path')

    path_pub.publish(local_path)


def pose_callback(msg):
    global pose
    pose = msg
    #rospy.loginfo('pose')
    #plan_path()

############################################################################

if __name__ == '__main__':
    rospy.init_node('motion_planner_node')

    listener = tf.TransformListener()
    rospy.Subscriber('/occupancy_grid', OccupancyGrid, grid_callback)
    rospy.Subscriber('/zed/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/reference_path', Path, path_callback)
    path_pub = rospy.Publisher('/local_path', Path, queue_size=1, latch=True)

    road_borders_helper = LinesHelper('/road_borders', [1,1,1,1])
    trajectories_helper = LinesHelper('/trajectories', [0.7,0.7,0.7,0.4])
    optimal_trajectory_helper = LinesHelper('/optimal_trajectory', [1, 0, 0,1])

    #rospy.spin()
    rate = rospy.Rate(1.0 / REPLANE_PERIOD)
    while not rospy.is_shutdown():
        plan_path()
        rate.sleep()
