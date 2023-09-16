std_msgs/Header header
float64 robot_time
geometry_msgs/Pose base_pos
geometry_msgs/Twist base_vel
sensor_msgs/Imu imu
sensor_msgs/MagneticField magnetic_sensor #TODO: rename??

# Actuated joints
digit_msgs/DigitMotors motor


# Unactuated joints
digit_msgs/DigitJoints joint

int16 battery_charge
