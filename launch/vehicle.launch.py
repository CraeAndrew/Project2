from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory("project2")  # Change the package name as needed


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="gps_nav",
                executable="route_pose_provider",
                name="route_pose_provider",
                parameters=[
                    {"want_loop": False},
                    {
                        "state_defs": "{0:'OFF', 1:'ON', 2:'OUTSIDE', 3:'ENTRY_EXTENSION_PT', 4:'EXIT_EXTENSION_PT', 5:'EXIT_TURN_PT', 6:'START', 7:'END', 8:'UTURN_PT1', 9:'UTURN_PT2', 10:'UTURN_PT3', 11:'CORNER', 12:'END_EXTENSION'}"
                    },
                    {
                        "pose_filename": config_dir + "/data/path.txt"
                    },  # Put your pose list in a folder called data within your package.
                ],
            ),
            Node(package="gps_nav", executable="goal_pose_creator", name="goal_pose_creator", output="screen"),
            Node(
                package="project2",
                executable="VehControl",
                name="VehControl",
                output="screen",
            ),
            Node(
                package="gps_nav",
                executable="motion_spec_provider",
                name="motion_spec_provider",
                output="screen",
                parameters=[{"look_ahead_dist": 3.0}, {"speed": 0.0}],  # meters  # meters/sec
            ),
            Node(
                package="odom_data",
                executable="publisher",
                name="publisher_node",
            ),
            Node(
                package="neo6m_gps",
                executable="driver",
                name="neo6m_driver_node",
            ),
        ]
    )
