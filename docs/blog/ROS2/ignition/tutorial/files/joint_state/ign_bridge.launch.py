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
    ign_model_prefix = "/world/demo/model/" + ROBOT_NAME

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
            ign_model_prefix + "/joint_state"
            + "@sensor_msgs/msg/JointState"
            + "[ignition.msgs.Model"
        ],
        remappings=[(ign_model_prefix + "/joint_state", "/joint_states")],
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(clock_bridge)
    ld.add_action(joint_state_bridge)
    return ld
