# Car state
# It is used to describe current car state or path-planning targets

std_msgs/Header header

# Position x-coordinate
float64 x
# Position y-coordinate
float64 y
# Car orientation around z-axis
float64 yaw

# Forward velocity
float64 velocity
# Angular velocity around z-axis
float64 yaw_velocity