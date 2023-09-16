from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory("kogrob2")

    parameter = os.path.join(
        get_package_share_directory('kogrob2'),
        'params.yaml'
        )


    feature_extractor = Node(package='kogrob2', executable='features_extractor_node')
    class_interaction = Node(package='kogrob2', executable='class_interaction_node', parameters = [parameter])
    rviz = Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', os.path.join(pkg_path +  "/config_task2.rviz")])

    return LaunchDescription([feature_extractor, class_interaction, rviz])
