from os import environ

from launch import LaunchDescription
from launch.actions import ExecuteProcess

    
exec="mavproxy.py"
ign_process = ExecuteProcess(
    cmd=[exec],
    emulate_tty=True
)

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(ign_process)
    return ld