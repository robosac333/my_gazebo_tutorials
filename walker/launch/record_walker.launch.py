from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers.on_process_start import OnProcessStart
from launch.substitutions import FindExecutable
import datetime

def generate_launch_description():
    # Get current timestamp for the bag file name
    timestamp = datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    
    # Walker node - updated executable name to match CMakeLists.txt
    walker_node = Node(
        package='walker',
        executable='robot_control',  # Changed from 'walker' to 'robot_control'
        name='walker',
        output='screen'
    )
    
    # ROS bag recording process
    rosbag_record = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' bag record',
            ' -a',                      # Record all topics
            ' --exclude "/camera/.*"',  # Exclude all camera topics
            ' -o walker_recording_',    # Output file prefix
            timestamp,                  # Add timestamp to filename
        ]],
        shell=True
    )
    
    # Timer to stop recording after 15 seconds
    timer_action = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='ros2'),
                    ' bag shutdown'
                ]],
                shell=True
            )
        ]
    )
    
    # Register event handler to start the timer when rosbag recording starts
    start_timer_event = RegisterEventHandler(
        OnProcessStart(
            target_action=rosbag_record,
            on_start=[timer_action]
        )
    )
    
    return LaunchDescription([
        walker_node,
        rosbag_record,
        start_timer_event
    ])