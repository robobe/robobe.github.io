from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, FindExecutable
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    ld = LaunchDescription()
    
    sim_node = Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
            respawn=True,
            respawn_delay=4
        )

    # ros2 service call /turtlesim1/spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2}"

    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            "/turtlesim1",
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )

    event_end_action = RegisterEventHandler(
            OnProcessExit(
                target_action=sim_node,
                on_exit=[
                    LogInfo(msg=("event --------" , 
                        EnvironmentVariable(name='USER'),
                        ' closed the turtlesim window'))
                ]
            )
        )

    event_start_action = RegisterEventHandler(
            OnProcessStart(
                target_action=sim_node,
                on_start=[
                    LogInfo(msg='event ------- Turtlesim started, spawning turtle -------'),
                    spawn_turtle
                ]
            )
        )

    ld.add_action(sim_node)
    ld.add_action(event_end_action)
    ld.add_action(event_start_action)
    
    return ld