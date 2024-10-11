from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace, SetRemap, Node, SetParameter


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
                                                        get_package_share_directory('eureka_navigation'),
                                                        'config',
                                                        'nav2.yaml'
                                                        ]),
                          description='Nav2 parameters'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace')
]


def launch_setup(context, *args, **kwargs):
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params_str = LaunchConfiguration('params_file').perform(context)
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str

    launch_nav2 = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(namespace_str + '/global_costmap/scan', namespace_str + '/scan'),
        SetRemap(namespace_str + '/local_costmap/scan', namespace_str + '/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                  ('use_sim_time', use_sim_time),
                  ('params_file', nav2_params_str),
                  ('use_composition', 'False'),
                  ('namespace', namespace_str)
                ]
        ),
    ])

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name="rtabmap",
        output="screen",
        parameters=[{
            "subscribe_depth": False,
            "subscribe_rgbd": False,
            "subscribe_rgb": False,
            "subscribe_stereo": False,
            "subscribe_scan": False,
            "subscribe_scan_cloud": True,
            "subscribe_user_data": False,
            "subscribe_odom_info": False,
            "frame_id": 'base_link',
            "map_frame_id": 'map',
            "odom_frame_id": 'odom',
            "publish_tf": True,
            "initial_pose": '',
            "use_action_for_goal": False,
            "ground_truth_frame_id": '',
            "ground_truth_base_frame_id": '',
            "odom_tf_angular_variance": 0.01,
            "odom_tf_linear_variance": 0.001,
            "odom_sensor_sync": False,
            "wait_for_transform": 0.2,
            "database_path": '~/.ros/rtabmap.db',
            "approx_sync": False,
            "config_path": '',
            "topic_queue_size": 10,
            "sync_queue_size": 10,
            "qos_image": 1,
            "qos_scan": 2,
            "qos_odom": 1,
            "qos_camera_info": 1,
            "qos_imu": 2,
            "qos_gps": 1,
            "qos_user_data": 1,
            "scan_normal_k": 0,
            "landmark_linear_variance": 0.0001,
            "landmark_angular_variance": 0.9999,
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "false",
            "Optimizer/Strategy" : "2",
            "Icp/Epsilon": "0.01",
            # "RGBD/ProximityPathMaxNeighbors": "0",
            "Optimizer/Robust" : "true",
            # "Optimizer/Strategy": "1"
        }],
        remappings=[
            ("/grid_prob_map", "/map"),
            ("scan_cloud", "/camera/points")
        ],
        arguments=["--delete_db_on_start"],
    )

    obstacle_detection = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacle_detection',
        output='screen',
        parameters=[{
            'frame_id' : 'base_link',
            'map_frame_id' : 'map'
        }],
        remappings=[
            ('/cloud', '/camera/points')
        ]
    )

    return [nav2,
            SetParameter(name='use_sim_time', value=True),
            rtabmap_slam
            # obstacle_detection
    ]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
