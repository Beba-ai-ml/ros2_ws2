from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Ścieżki do plików konfiguracyjnych
    pkg_share = get_package_share_directory('f1tenth_stack')
    joy_teleop_config = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')
    vesc_config       = os.path.join(pkg_share, 'config', 'vesc.yaml')
    mux_config        = os.path.join(pkg_share, 'config', 'mux.yaml')
    local_python_path = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'local_python')
    pythonpath_env = os.pathsep.join(
        [local_python_path, os.environ.get('PYTHONPATH', '')]
    ) if os.environ.get('PYTHONPATH') else local_python_path

    # ======================
    # ARGUMENTY F1TENTH
    # ======================
    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Path to joy_teleop config file'
    )
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Path to VESC config file'
    )
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Path to ackermann mux config file'
    )

    # ======================
    # ARGUMENTY LIDARA (S1 – sllidar_ros2)
    # ======================
    channel_type      = LaunchConfiguration('channel_type')
    serial_port       = LaunchConfiguration('serial_port')
    serial_baudrate   = LaunchConfiguration('serial_baudrate')
    frame_id          = LaunchConfiguration('frame_id')
    inverted          = LaunchConfiguration('inverted')
    angle_compensate  = LaunchConfiguration('angle_compensate')
    scan_mode         = LaunchConfiguration('scan_mode')

    channel_type_la = DeclareLaunchArgument(
        'channel_type',
        default_value='serial',
        description='Specifying channel type of lidar (serial/tcp)'
    )

    serial_port_la = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Specifying usb port to connected lidar'
    )

    # Uwaga: S1 ma typowe 256000 baud
    serial_baudrate_la = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='256000',
        description='Specifying usb port baudrate to connected lidar'
    )

    frame_id_la = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Specifying frame_id of lidar'
    )

    inverted_la = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Specifying whether or not to invert scan data'
    )

    angle_compensate_la = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Specifying whether or not to enable angle_compensate of scan data'
    )

    # Dla S1 zwykle działa "Standard" – jakby marudził na tryb, można zmienić z CLI
    scan_mode_la = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Specifying scan mode of lidar'
    )

    ld = LaunchDescription()
    # F1TENTH args
    ld.add_action(joy_la)
    ld.add_action(vesc_la)
    ld.add_action(mux_la)
    # LIDAR args
    ld.add_action(channel_type_la)
    ld.add_action(serial_port_la)
    ld.add_action(serial_baudrate_la)
    ld.add_action(frame_id_la)
    ld.add_action(inverted_la)
    ld.add_action(angle_compensate_la)
    ld.add_action(scan_mode_la)

    # ======================
    # 1. JOYSTICK (joy_linux)
    # ======================
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.01,
            'autorepeat_rate': 20.0,
            'coalesce_interval': 0.01
        }]
    )

    # ======================
    # 2. JOY_TELEOP
    # ======================
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[LaunchConfiguration('joy_config')],
        env=dict(os.environ, PYTHONPATH=pythonpath_env)
    )

    # ======================
    # 2b. JOY MODE MANAGER (gates teleop + autonomy lock from joy buttons)
    # ======================
    joy_mode_manager_node = Node(
        package='f1tenth_stack',
        executable='joy_mode_manager',
        name='joy_mode_manager',
        output='screen'
    )

    # ======================
    # 3. VESC + ODOM
    # ======================
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    # Jeśli chcesz wygładzać gaz/skręt, odkomentuj:
    # throttle_interpolator_node = Node(
    #     package='f1tenth_stack',
    #     executable='throttle_interpolator',
    #     name='throttle_interpolator',
    #     output='screen',
    #     parameters=[LaunchConfiguration('vesc_config')]
    # )

    # ======================
    # 4. ACKERMANN MUX
    # ======================
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        output='screen',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[
            ('ackermann_cmd_out', 'ackermann_drive')
        ]
    )

    # ======================
    # 5. Statyczny TF base_link -> laser
    # ======================
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=[
            '0.27', '0.0', '0.11',   # x y z
            '0.0', '0.0', '0.0',     # roll pitch yaw
            'laser', 'base_link'
        ],
        output='screen'
    )

    # ======================
    # 6. LIDAR S1 (sllidar_ros2)
    # ======================
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type':      channel_type,
            'serial_port':       serial_port,
            'serial_baudrate':   serial_baudrate,
            'frame_id':          frame_id,
            'inverted':          inverted,
            'angle_compensate':  angle_compensate,
            'scan_mode':         scan_mode,
        }]
    )

    # Dodajemy wszystkie nody do launch description
    for node in [
        joy_node,
        joy_teleop_node,
        joy_mode_manager_node,
        ackermann_to_vesc_node,
        vesc_to_odom_node,
        vesc_driver_node,
        # throttle_interpolator_node,
        ackermann_mux_node,
        static_tf_node,
        lidar_node,
    ]:
        ld.add_action(node)

    return ld
