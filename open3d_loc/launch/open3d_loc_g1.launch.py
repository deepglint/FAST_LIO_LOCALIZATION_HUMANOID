from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    open3d_loc_share = FindPackageShare('open3d_loc')

    # 声明 use_sim_time 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # 配置文件路径
    config_file = PathJoinSubstitution([
        open3d_loc_share,
        'config',
        'loc_param_g1.yaml'
    ])

    # 地图文件路径 - 使用绝对路径指向源码目录中的地图文件
    map_file = '/home/sax/GO2_Localization_ROS2/src/GO2_Localization_ROS2/data/1.test.ply'

    # 静态TF发布节点 - camera_init to odom
    static_tf_camera_init2odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init2odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init']
    )

    # 静态TF发布节点 - imu_link to base_link
    # 修正：父frame是imu_link，子frame是base_link
    static_tf_imulink2baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imulink2baselink',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'imu_link', 'base_link']
    )

    # 静态TF发布节点 - base_link to motion_link
    # 修正：base_link是父frame，motion_link是子frame
    static_tf_base_center = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_center_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0',
                   '1', 'base_link', 'motion_link']
    )

    # 全局定位节点
    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[
            config_file,
            {
                'path_map': map_file,
                'pcd_queue_maxsize': 10,
                'voxelsize_coarse': 0.01,
                'voxelsize_fine': 0.2,
                'threshold_fitness': 0.5,
                'threshold_fitness_init': 0.5,
                'loc_frequence': 2.5,
                'save_scan': False,
                'hidden_removal': False,
                'maxpoints_source': 80000,
                'maxpoints_target': 400000,
                'filter_odom2map': False,
                'kalman_processVar2': 0.001,
                'kalman_estimatedMeasVar2': 0.02,
                'confidence_loc_th': 0.7,
                'dis_updatemap': 3.5,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # 点云转换节点
    pointcloud_transformer_node = Node(
        package='open3d_loc',
        executable='pointcloud_transformer_node',
        name='pointcloud_transformer_node',
        output='screen',
        parameters=[{
            'input_topic': '/cloud_registered_body_1',
            'output_topic': '/cloud_registered_map',
            'global_map_topic': '/global_map',
            'source_frame': 'base_link',
            'target_frame': 'map',
            'voxel_leaf_size': 0.1,
            'map_voxel_leaf_size': 0.2,
            'max_global_points': 1000000,
            'map_publish_frequency': 1.0,
            'enable_global_map': True,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        static_tf_camera_init2odom,
        static_tf_imulink2baselink,
        static_tf_base_center,
        global_localization_node,
        # pointcloud_transformer_node
    ])
