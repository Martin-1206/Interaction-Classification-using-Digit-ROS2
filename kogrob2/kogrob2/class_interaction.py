

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
#from rosbag2_py.
from ros2topic.api import get_topic_names_and_types
import math
import numpy as np



class ClassInteraction(Node):
    """
    class functions can be defined below the constructor
    """

    def __init__(self) -> None:
        """
        defnition of the constructor of the class
        """
        super().__init__('class_interaction_node')       # node name

        self.declare_parameters(
            namespace='',
            parameters=[
                ('distance_x', 0.0),
                ('velocity', 0.0),
                ('FoV', 0.0)
            ])

        

        self.thresh_distance = self.get_parameter('distance_x').get_parameter_value().double_value   # get paramter von yaml
        self.thresh_velocity = self.get_parameter('velocity').get_parameter_value().double_value
        self.thresh_FoV = self.get_parameter('FoV').get_parameter_value().double_value
        self.features_subscriber = self.create_subscription(Float32MultiArray, '/features', self.features_callback, 10) # subscriber 

        self.FoV_Good = False
        self.Distance_good = False
        self.Digit_1_Zero_Velocity = False
        self.Digit_2_Zero_Velocity = False
        self.Interaction = False



    def features_callback(self, msg: Float32MultiArray):

        current_x = msg.data[0]     # first eintrag in array
        current_y = msg.data[1]
        current_z = msg.data[2]
        current_yaw = msg.data[3]

        current_yaw = np.absolute(np.rad2deg(current_yaw))  # turns current Yaw into degree and amount(Betrag)
        
        
        if ((180 - current_yaw) < self.thresh_FoV):
            if self.FoV_Good == False:
                self.get_logger().info(f'FoV is good')      #funktioniert
                self.FoV_Good = True
            
        else:
            if self.FoV_Good == True:
                self.get_logger().info(f'FoV is not good')  #funktioniert
                self.FoV_Good = False

        

    
        

        distance_digit1_digit2 = np.linalg.norm([current_x, current_y]) 

        if distance_digit1_digit2 < self.thresh_distance:
            if self.Distance_good == False:
                self.get_logger().info(f'Distance is good') 
                self.Distance_good = True
        else:
            if self.Distance_good == True:
                self.get_logger().info(f'Distance is not good') 
                self.Distance_good = False

                

        linear_velocity_digit_1 = msg.data[4]
        angular_velocity_digit_1 = msg.data[5]

        linear_velocity_digit_2 = msg.data[6]
        angular_velocity_digit_2 = msg.data[7]

  
        if (linear_velocity_digit_1 < self.thresh_velocity) and (angular_velocity_digit_1 < self.thresh_velocity):
            if self.Digit_1_Zero_Velocity == False:
                self.get_logger().info(f'Digit 1 Zero Velocity')    
                self.Digit_1_Zero_Velocity = True
        else:
            if self.Digit_1_Zero_Velocity == True:
                self.get_logger().info(f'Digit 1 has Velocity')     
                self.Digit_1_Zero_Velocity = False
            

        if (linear_velocity_digit_2 < self.thresh_velocity) and (angular_velocity_digit_2 < self.thresh_velocity):
                if self.Digit_2_Zero_Velocity == False:
                    self.get_logger().info(f'Digit 2 Zero Velocity')    
                    self.Digit_2_Zero_Velocity = True
        else:
            if self.Digit_2_Zero_Velocity == True:
                self.get_logger().info(f'Digit 2 has Velocity')     
                self.Digit_2_Zero_Velocity = False
            

        if ((180 - current_yaw) < self.thresh_FoV) and (distance_digit1_digit2 < self.thresh_distance) and (linear_velocity_digit_1 < self.thresh_velocity) and (angular_velocity_digit_1 < self.thresh_velocity) and (linear_velocity_digit_2 < self.thresh_velocity) and (angular_velocity_digit_2 < self.thresh_velocity):
            if self.Interaction == False:
                self.get_logger().info(f'INTERACTION')   
                self.Interaction = True
        else:
            if self.Interaction == True:
                self.get_logger().info(f'NO INTERACTION') 
                self.Interaction = False
      
        
        
        
            

def main(args=None):

    rclpy.init(args=args)
    class_interaction = ClassInteraction()

    try: 
        rclpy.spin(class_interaction)
    except KeyboardInterrupt:
        pass

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
if __name__ == '__main__':
    main()




        