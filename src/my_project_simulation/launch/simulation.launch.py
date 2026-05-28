from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    startup_pkg_dir = get_package_share_directory('my_project_startup')
    startup_launch_path = os.path.join(startup_pkg_dir, 'launch', 'startup.launch.py')

    # Declare npz_path argument so it can be passed from CLI
    npz_path_arg = DeclareLaunchArgument(
        'npz_path',
        default_value='/home/jayson/kuka_ram_ws/data/output_npz/test.npz',
        description='Path to NPZ file or manifest'
    )

    # Socat process to create virtual serial port pair
    # Using pty,raw,echo=0 to avoid terminal echoing
    socat_process = ExecuteProcess(
        cmd=['socat', '-d', '-d', 'pty,raw,echo=0,link=/tmp/ttyV0', 'pty,raw,echo=0,link=/tmp/ttyV1'],
        output='screen'
    )

    # Launch rqt GUI
    rqt_process = ExecuteProcess(
        cmd=['rqt', '--standalone', 'my_project_ui/UiPanel'],
        output='screen'
    )

    # Launch the main system with overridden parameters for simulation
    startup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(startup_launch_path),
        launch_arguments={
            'port': '/tmp/ttyV0',
            'local_ip': '127.0.0.1',
            'local_port': '49152',
            'npz_path': LaunchConfiguration('npz_path')
        }.items()
    )

    # Mock KUKA Node
    mock_kuka_node = Node(
        package='my_project_simulation',
        executable='mock_kuka',
        name='mock_kuka',
        output='screen',
        parameters=[{
            'remote_ip': '127.0.0.1',
            'remote_port': 49152
        }]
    )

    # Mock MCU Node
    mock_mcu_node = Node(
        package='my_project_simulation',
        executable='mock_mcu',
        name='mock_mcu',
        output='screen',
        parameters=[{
            'port': '/tmp/ttyV1',
            'baudrate': 115200
        }]
    )

    return LaunchDescription([
        npz_path_arg,
        socat_process,
        rqt_process,
        mock_mcu_node,
        mock_kuka_node,
        startup_launch,
    ])
