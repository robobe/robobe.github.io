import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "ign_tutorial"
SDF_MODEL_NAME = "vehicle_2"
WORLD_NAME = "vehicle.sdf"

def generate_launch_description():
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=["true"],
        description="Enable sim time from /clock",
    )
    with_bridge_arg = DeclareLaunchArgument(
        "with_bridge",
        default_value=["false"],
        description="Launch simulation with ros ign bridge",
    )

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg = get_package_share_directory(PACKAGE_NAME)

    sdf_path = f"{pkg}/models/{SDF_MODEL_NAME}/model.sdf"
    use_sim_time = LaunchConfiguration("use_sim_time")

    resources = [os.path.join(pkg, "worlds"), os.path.join(pkg, "models")]
    resource_env = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 2 {WORLD_NAME}"}.items(),
    )

    # launch ign_bridge if with_bridge is true
    ign_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "ign_bridge.launch.py"),
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration("with_bridge"))
    )

    # robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": open(sdf_path).read()},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(with_bridge_arg)

    ld.add_action(resource_env)
    ld.add_action(gazebo)
    ld.add_action(ign_bridge)
    ld.add_action(robot_state_publisher)
    return ld
