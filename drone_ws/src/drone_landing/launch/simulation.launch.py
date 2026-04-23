import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('drone_landing')

    drone_urdf = xacro.process_file(
        os.path.join(pkg, 'urdf', 'drone.urdf.xacro')).toxml()
    plat_urdf = xacro.process_file(
        os.path.join(pkg, 'urdf', 'platform.urdf.xacro')).toxml()

    world_file  = os.path.join(pkg, 'worlds', 'landing_world.world')
    rviz_config = os.path.join(pkg, 'rviz',   'drone_landing.rviz')

    # ── Gazebo ──────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen')

    # ── State publishers (start immediately) ────────────────────────
    drone_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='drone_state_publisher',
        parameters=[{'robot_description': drone_urdf,
                     'publish_frequency': 30.0}],
        remappings=[('/joint_states', '/drone/joint_states')])

    platform_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='platform_state_publisher',
        namespace='platform',
        parameters=[{'robot_description': plat_urdf,
                     'publish_frequency': 30.0}])

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_tf',
        arguments=['--x','0','--y','0','--z','0',
                   '--roll','0','--pitch','0','--yaw','0',
                   '--frame-id','odom',
                   '--child-frame-id','base_link'])

    # RViz
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen')

    # ── Spawn drone at t=7s (increased from 5s — gives Gazebo more time)
    spawn_drone = TimerAction(period=7.0, actions=[
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', '/robot_description',
                        '-entity', 'quadrotor',
                        '-x', '0', '-y', '0', '-z', '4.5'],
             output='screen')])

    # ── Spawn platform at t=11s
    spawn_platform = TimerAction(period=11.0, actions=[
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', '/platform/robot_description',
                        '-entity', 'landing_platform',
                        '-x', '2', '-y', '0', '-z', '0.025'],
             output='screen')])

    # ── Platform controller at t=14s
    platform_ctrl = TimerAction(period=14.0, actions=[
        Node(package='drone_landing', executable='platform_controller',
             name='platform_controller',
             parameters=[{'pattern':    'sine',
                          'amplitude':   1.5,
                          'z_amplitude': 0.3,
                          'frequency':   0.15}],
             output='screen')])

    # ── ArUco detector at t=16s
    aruco = TimerAction(period=16.0, actions=[
        Node(package='drone_landing', executable='aruco_detector',
             name='aruco_detector',
             remappings=[
                 ('/drone/camera/image_raw',
                  '/drone/downward_camera/image_raw'),
                 ('/drone/camera/camera_info',
                  '/drone/downward_camera/camera_info'),
             ],
             output='screen')])

    # ── Landing controller at t=18s
    landing_ctrl = TimerAction(period=18.0, actions=[
        Node(package='drone_landing', executable='landing_controller',
             name='landing_controller', output='screen')])

    # ── State machine at t=20s (last — needs everything ready)
    state_machine = TimerAction(period=20.0, actions=[
        Node(package='drone_landing', executable='state_machine',
             name='state_machine', output='screen')])

    return LaunchDescription([
        gazebo,
        drone_state_pub,
        platform_state_pub,
        static_tf,
        rviz,
        spawn_drone,
        spawn_platform,
        platform_ctrl,
        aruco,
        landing_ctrl,
        state_machine,
    ])
