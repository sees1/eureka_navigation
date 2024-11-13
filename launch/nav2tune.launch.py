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
    DeclareLaunchArgument('use_sim_time', default_value='false',
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
        SetRemap(namespace_str + '/cmd_vel', namespace_str + '/custom_cmd_vel'),


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

    rgbd_parameters = [{
        "frame_id": "base_footprint",
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "approx_sync": True,
        "approx_sync_max_interval": 0.01,
        "publish_tf": False,
        "wait_imu_to_init": False,
        "publish_null_when_lost": False,
        "qos": 2,
        "qos_camera_info": 2,

        # 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
        "Optimizer/Strategy": "2",
        "Optimizer/GravitySigma": "0.3",

        # 0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D
        "Odom/Strategy": "0",
        "Odom/ResetCountdown": "1",
        "Odom/Holonomic": "false",
        # 0=No filtering 1=Kalman filtering 2=Particle filtering
        "Odom/FilteringStrategy": "0",
        "Odom/ParticleSize": "400",
        "Odom/GuessMotion": "true",
        "Odom/AlignWithGround": "false",

        "GFTT/MinDistance": "5.0",
        "GFTT/QualityLevel": "0.001",
        "GFTT/BlockSize": "4",
        "GFTT/UseHarrisDetector": "false",
        "GFTT/K": "0.04",

        "SURF/Extended": "true",
        "SURF/HessianThreshold": "500",
        "SURF/Octaves": "4",
        "SURF/OctaveLayers": "4",
        "SURF/Upright": "false",
        "SURF/GpuVersion": "false",
        "SURF/GpuKeypointsRatio": "0.01",

        "SIFT/NFeatures": "1000",
        "SIFT/NOctaveLayers": "4",
        "SIFT/RootSIFT": "false",

        "FREAK/OrientationNormalized": "true",
        "FREAK/ScaleNormalized": "true",
        "FREAK/PatternScale": "30",
        "FREAK/NOctaves": "4",

        "KAZE/Extended": "true",
        "KAZE/Upright": "false",
        "KAZE/NOctaves": "4",
        "KAZE/NOctaveLayers": "4",
        # 0=DIFF_PM_G1, 1=DIFF_PM_G2, 2=DIFF_WEICKERT or 3=DIFF_CHARBONNIER
        "KAZE/Diffusivity": "1",

        "BRIEF/Bytes": "64",

        "Vis/EstimationType": "1",
        "Vis/ForwardEstOnly": "true",
        # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector
        "Vis/FeatureType": "8",
        "Vis/DepthAsMask": "true",
        "Vis/CorGuessWinSize": "40",
        "Vis/MaxFeatures": "1000",
        "Vis/MinDepth": "0.0",
        "Vis/MaxDepth": "0.0",
        # 0=Features Matching, 1=Optical Flow
        "Vis/CorType": "0",
        # kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7
        "Vis/CorNNType": "1"
    }]

    rgbd_odometry = Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            output="log",
            parameters=rgbd_parameters,
            remappings=[
                ("rgb/image", "camera/camera/color/image_raw"),
                ("rgb/camera_info", "camera/camera/color/camera_info"),
                ("depth/image", "camera/camera/depth/image_rect_raw"),
                ("imu", "imu/data"),
                ("odom", "odom_rgbd")
            ],
            arguments=["--ros-args", "--log-level", "Error"]
    )

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name="rtabmap",
        output="screen",
        parameters=[{
            "subscribe_depth": True,
            "subscribe_rgbd": False,
            "subscribe_rgb": False,
            "subscribe_stereo": False,
            "subscribe_scan": False,
            "subscribe_scan_cloud": True,
            "subscribe_user_data": False,
            "subscribe_odom_info": False,
            "frame_id": 'base_footprint',
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
            "qos_image": 2,
            "qos_scan": 2,
            "qos_odom": 1,
            "qos_camera_info": 1,
            "qos_imu": 0,
            "qos_gps": 0,
            "qos_user_data": 1,
            "scan_normal_k": 0,
            "landmark_linear_variance": 0.0001,
            "landmark_angular_variance": 0.9999,
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "false",
            "Optimizer/Strategy" : "2",
            "Icp/Epsilon": "0.01",
            # "RGBD/ProximityPathMaxNeighbors": "0",
            "RGBD/OptimizeStrategy": "2",
            "RGBD/OptimizeRobust": "true",
            "RGBD/OptimizeMaxError": "0",
            "Optimizer/Robust" : "true",
            # "Optimizer/Strategy": "1"
        }],
        remappings=[
            ("/grid_prob_map", "/map"),
            ("rgb/image", "camera/camera/color/image_raw"),
            ("rgb/camera_info", "camera/camera/color/camera_info"),
            ("depth/image", "camera/camera/depth/image_rect_raw"),
            ("scan_cloud", "/camera/camera/depth/color/points")
        ],
        arguments=["--delete_db_on_start"],
    )

    return [nav2,
            SetParameter(name='use_sim_time', value=True),
            rtabmap_slam,
            # rgbd_odometry
    ]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
