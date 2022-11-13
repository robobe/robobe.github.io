from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_NAME = "basic_mobile_bot"

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value=["false"], description="use sim time from /clock"
    )

    namespace = ""
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_model_prefix = "/model/" + ROBOT_NAME
    full_ign_model_prefix = "/world/demo" + ign_model_prefix

    # clock bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        condition=IfCondition(use_sim_time),
    )

    # joint state bridge
    # /world/demo/model/v2/model/basic_mobile_bot/joint_state
    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        name="joint_state_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            full_ign_model_prefix + "/joint_state"
            + "@sensor_msgs/msg/JointState"
            + "[ignition.msgs.Model"
        ],
        remappings=[(full_ign_model_prefix + "/joint_state", "/joint_states")],
    )

    # cmd_vel bridge 
    cmd_vel_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
            namespace = namespace,
            name = 'cmd_vel_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            arguments = [
                ign_model_prefix + '/cmd_vel' + '@geometry_msgs/msg/Twist' + ']ignition.msgs.Twist'
            ],
            remappings = [
                (ign_model_prefix + '/cmd_vel', '/cmd_vel')
            ])

    # odometry bridge 
    odometry_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
            namespace = namespace,
            name = 'odometry_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            arguments = [
                    ign_model_prefix + '/odometry' + '@nav_msgs/msg/Odometry' + '[ignition.msgs.Odometry'
            ],
            remappings = [
                (ign_model_prefix + '/odometry', '/odom')
            ])

    # odom to base_link transform bridge
    odom_base_tf_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
            namespace = namespace,
            name = 'odom_base_tf_bridge',
            output = 'screen',
            parameters=[{
            'use_sim_time': use_sim_time
            }],
            arguments = [
                ign_model_prefix + '/tf' + '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V'
            ],
            remappings = [
                (ign_model_prefix + '/tf', '/tf')
            ])

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'qos_overrides./imu.publisher.reliability': 'best_effort'
        }]
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="imu2imu_link",
        arguments = ["0", "0", "0", "0", "0", "0", "imu_link", "basic_mobile_bot/imu_link/imu"]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(clock_bridge)
    ld.add_action(joint_state_bridge)
    ld.add_action(odometry_bridge)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(odom_base_tf_bridge)
    ld.add_action(imu_bridge)
    ld.add_action(imu_tf)
    return ld
