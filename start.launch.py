from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():
    ld = LaunchDescription(
        [
        # declares the rosbag_in parameter as a command line argument
        DeclareLaunchArgument(
            'bag_in',
            default_value='default_bag_in',
            description='Input bag file'
        ),
        # declares the rosbag_out parameter as a command line argument
        DeclareLaunchArgument(
            'bag_out',
            default_value='default_bag_out',
            description='Output bag file'
        ),
        
        ]
    )



    scan_subscriber_node = Node(
        package='project3',
        executable='scan_subscriber_node',
        )


    count_and_track_node = Node(
        package='project3',
        executable='count_and_track_node',
    )



    # this plays the bag file based off the input file
    play_bag = ExecuteProcess(
        cmd=[[
            'ros2 bag play ',
            LaunchConfiguration('bag_in'),
        ]],
        shell=True
    )


    #this records the bag file to the correct output file.

    record_bag = ExecuteProcess(
        name="record_bag",
        cmd=[[
            'ros2 bag record -a --storage sqlite3 -o ',
            LaunchConfiguration('bag_out'),
        ]],
        shell=True
    )



    
    ld.add_action(record_bag)

    ld.add_action(play_bag)
    
    ld.add_action(scan_subscriber_node)

    ld.add_action(count_and_track_node)

    
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=play_bag,
            on_exit=[
                LogInfo(msg=('Finished playing bag file')),
                
                # this shuts down the running nodes
                EmitEvent(event=Shutdown(
                    reason='Finished bag file')),
                ]
            )
        ))



    
    return ld

    