


import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float32MultiArray
from ros2topic.api import get_topic_names_and_types
import math
import numpy as np
from digit_msgs.msg import DigitState
import yaml 

class FrameListener(Node):
    """
    class functions can be defined below the constructor
    """

    def __init__(self) -> None:
        """
        defnition of the constructor of the class
        """
        super().__init__('features_extractor_node')       # node name

        
        self.tf_buffer = Buffer()                                   # create buffer
        self.tf_listener = TransformListener(self.tf_buffer, self)  # create listener       


        self.features = self.create_publisher(Float32MultiArray, 'features', 10)    # create publisher

        '''Next, a timer is created with a callback to execute every 1 seconds. self.i is a counter used in the callback.'''
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_z   = 0.0
        self.current_yaw = 0.0

        self.linear_velocity_digit_1  = 0.0
        self.angular_velocity_digit_1 = 0.0

        self.linear_velocity_digit_2  = 0.0
        self.angular_velocity_digit_2 = 0.0


        self.digits_found = False
        self.digit_list = []
        

    def timer_callback(self):

        self.find_digits()
        self.assign_digit_names()
            
        self.extract_features()
        self.digit_1 = self.create_subscription(DigitState, self.TFdigit_1 + '/digit_state', self.digit_1_callback, 10)
        self.digit_2 = self.create_subscription(DigitState, self.TFdigit_2 + '/digit_state', self.digit_2_callback, 10) 
        

    def find_digits(self):
        
        dic =yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        
        for key in dic:
            if dic[key]['parent'] == "map":
                key_split = key.split("/")[0]
                self.digit_list.append(key_split)

        

    def assign_digit_names(self):
        self.TFdigit_1 = self.digit_list[0]         
        self.TFdigit_2 = self.digit_list[1]

        

    def extract_features(self):


        from_frame_rel = self.TFdigit_1 + '/base_link'
        to_frame_rel = self.TFdigit_2 + '/base_link'        


        try:
            t = self.tf_buffer.lookup_transform(from_frame_rel, to_frame_rel, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        ''' Publish Pose'''
        self.current_x = t.transform.translation.x
        self.current_y = t.transform.translation.y
        self.current_z = t.transform.translation.z
        yaw = self.euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
        self.current_yaw = yaw
        
        msg = Float32MultiArray()
        msg.data = [self.current_x, self.current_y, self.current_z, self.current_yaw, self.linear_velocity_digit_1, self.angular_velocity_digit_1, self.linear_velocity_digit_2, self.angular_velocity_digit_2]

        self.features.publish(msg)      # publish data


    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """ 
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return yaw_z # in radians

    def digit_1_callback(self, msg: DigitState):    
        
        velocity_digit_1_linear_x = msg.base_vel.linear.x
        velocity_digit_1_linear_y = msg.base_vel.linear.y
        velocity_digit_1_linear_z = msg.base_vel.linear.z

        self.linear_velocity_digit_1 = np.linalg.norm([velocity_digit_1_linear_x, velocity_digit_1_linear_y, velocity_digit_1_linear_z]) # Norm of vector

        velocity_digit_1_angular_x = msg.base_vel.angular.x
        velocity_digit_1_angular_y = msg.base_vel.angular.y
        velocity_digit_1_angular_z = msg.base_vel.angular.z

        self.angular_velocity_digit_1 = np.linalg.norm([velocity_digit_1_angular_x, velocity_digit_1_angular_y, velocity_digit_1_angular_z]) # Norm of vector


    def digit_2_callback(self, msg: DigitState):

        velocity_digit_2_linear_x = msg.base_vel.linear.x
        velocity_digit_2_linear_y = msg.base_vel.linear.y
        velocity_digit_2_linear_z = msg.base_vel.linear.z

        self.linear_velocity_digit_2 = np.linalg.norm([velocity_digit_2_linear_x, velocity_digit_2_linear_y, velocity_digit_2_linear_z]) # Norm of vector

        velocity_digit_2_angular_x = msg.base_vel.angular.x
        velocity_digit_2_angular_y = msg.base_vel.angular.y
        velocity_digit_2_angular_z = msg.base_vel.angular.z

        self.angular_velocity_digit_2 = np.linalg.norm([velocity_digit_2_angular_x, velocity_digit_2_angular_y, velocity_digit_2_angular_z]) # Norm of vector   



def main(args=None):

    rclpy.init(args=args)
    features_extractor = FrameListener()

    try: 
        rclpy.spin(features_extractor)
    except KeyboardInterrupt:
        pass

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
if __name__ == '__main__':
    main()

        
        
        
