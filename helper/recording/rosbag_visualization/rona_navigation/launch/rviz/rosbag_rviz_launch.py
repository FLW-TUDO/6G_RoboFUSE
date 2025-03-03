# from launch import LaunchDescription
# from launch_ros.actions import Node
# import os

# def generate_launch_description():
#     rviz_config_file = os.path.join(os.getenv('HOME'), '.rviz2/rosbag_play.rviz')

#     return LaunchDescription([
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             parameters=[{'use_sim_time': True}],  # ✅ Force use_sim_time
#             arguments=['-d', rviz_config_file],   # ✅ Load your RViz config
#         )
#     ])

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, TextSubstitution
# import os

# def generate_launch_description():
#     # Declare namespace argument (default: ep03)
#     declare_namespace_cmd = DeclareLaunchArgument(
#         'namespace',
#         default_value=TextSubstitution(text='ep03'),
#         description='Namespace for the robot (e.g., ep03, ep05)'
#     )

#     namespace = LaunchConfiguration('namespace')
#     #rviz_file = '.rviz2/rosbag_play_updated_' + namespace + '.rviz'
#     rviz_file = f'.rviz2/rosbag_play_{namespace}.rviz'

#     # Load RViz2 configuration file
#     rviz_config_file = os.path.join(os.getenv('HOME'), rviz_file) #'.rviz2/rosbag_play_updated.rviz'

#     return LaunchDescription([
#         declare_namespace_cmd,
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             parameters=[
#                 {'use_sim_time': True},  # ✅ Enable simulation time
#                 {'namespace': namespace}  # ✅ Pass namespace as a parameter
#             ],
#             arguments=['-d', rviz_config_file],  # ✅ Load custom RViz config
#         )
#     ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)  # ✅ Get evaluated namespace

    # Construct RViz config file path dynamically
    rviz_file = f'.rviz2/rosbag_play_{namespace}.rviz'
    rviz_config_file = os.path.join(os.getenv('HOME'), rviz_file)

    # Check if the RViz file exists (prevent RViz from failing)
    if not os.path.exists(rviz_config_file):
        print(f"⚠️ Warning: RViz config file not found: {rviz_config_file}. Using default.")
        rviz_config_file = os.path.join(os.getenv('HOME'), '.rviz2/rosbag_play.rviz')

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                {'use_sim_time': True},  # ✅ Enable simulation time
                {'namespace': namespace}  # ✅ Pass namespace as a parameter
            ],
            arguments=['-d', rviz_config_file],  # ✅ Load corresponding RViz config
        )
    ]

def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=TextSubstitution(text='ep03'),
        description='Namespace for the robot (e.g., ep03, ep05)'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        OpaqueFunction(function=launch_setup)  # ✅ Ensures LaunchConfiguration is evaluated correctly
    ])
