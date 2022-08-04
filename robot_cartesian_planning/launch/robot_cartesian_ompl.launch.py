from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('iiwa_description'), 'config', 'iiwa.config.xacro']
            ),
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Get SRDF via xacro
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('iiwa_description'), 'srdf', 'iiwa.srdf.xacro']
            ),
            ' ',
            'name:=',
            'iiwa',
        ]
    )

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    kinematics_yaml = PathJoinSubstitution([
            FindPackageShare('iiwa_description'),
            'moveit2',
            'kinematics.yaml'
        ]
    )

    robot_cartesian_config = PathJoinSubstitution([
            FindPackageShare('robot_cartesian_planning'),
            'config',
            'robot_cartesian_config.yaml'
        ]
    )

    move_group_interface = Node(
        name="robot_cartesian_ompl",
        package="robot_cartesian_planning",
        executable="robot_cartesian_ompl",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            robot_cartesian_config
        ],
    )

    return LaunchDescription([
            move_group_interface
        ]
    )
