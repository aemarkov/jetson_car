# Array of states that represents car path in a SE(2) state space
# State:
#   x, y, yaw
#   dx/dt, dy/dt, dyaw/dt

std_msgs/Header header
jetson_car/CarState[] states