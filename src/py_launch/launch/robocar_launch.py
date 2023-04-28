import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package = 'cont_cmdvel',
            executable = 'exec_cmd_vel',
            name = 'cmdvel_node'),
        launch_ros.actions.Node(
            package = 'sub_odom',
            executable = 'exec_odom',
            name = 'odom_node'),
    ])
